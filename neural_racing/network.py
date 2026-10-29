"""Deep Residual Feedforward Policy and Adam optimizer in plain NumPy."""

import numpy as np

DEFAULT_HIDDEN_DIMS = (128, 128, 128, 128, 128, 64)
LEAKY_SLOPE = 0.05


class Policy:
    def __init__(self, obs_dim=19, hidden_dims=DEFAULT_HIDDEN_DIMS, act_dim=2, seed=0):
        rng = np.random.default_rng(seed)
        if isinstance(hidden_dims, (int, np.integer)):
            hidden_dims = (int(hidden_dims),) * 6
        self.obs_dim = int(obs_dim)
        self.hidden_dims = tuple(int(h) for h in hidden_dims)
        self.act_dim = int(act_dim)
        self.num_layers = len(self.hidden_dims) + 1

        dims = [self.obs_dim] + list(self.hidden_dims) + [self.act_dim]
        self.params = {}

        for i in range(1, self.num_layers):
            in_d, out_d = dims[i - 1], dims[i]
            # Lower initial scale on residual blocks keeps deep forward variance stable
            scale = np.sqrt(1.0 / in_d) if in_d == out_d else np.sqrt(2.0 / in_d)
            self.params[f"W{i}"] = rng.standard_normal((in_d, out_d)) * scale
            self.params[f"b{i}"] = np.zeros(out_d, dtype=np.float64)

        last = self.num_layers
        self.params[f"W{last}"] = rng.uniform(-0.01, 0.01, (dims[-2], dims[-1]))
        self.params[f"b{last}"] = np.zeros(dims[-1], dtype=np.float64)
        # Mild forward throttle bias so the untrained car drives forward immediately
        self.params[f"b{last}"][1] = np.arctanh(0.55)

        self.last_layer_activations = []

    def forward(self, obs):
        p = self.params
        x = np.atleast_2d(obs).astype(np.float64)
        pre_acts = []
        post_acts = [x]

        h = x
        for i in range(1, self.num_layers):
            W, b = p[f"W{i}"], p[f"b{i}"]
            u = h @ W + b
            a = np.where(u > 0.0, u, LEAKY_SLOPE * u)
            # Residual skip connection when dimensions match
            h = a + h if h.shape[1] == a.shape[1] else a
            pre_acts.append(u)
            post_acts.append(h)

        W_last, b_last = p[f"W{self.num_layers}"], p[f"b{self.num_layers}"]
        z = h @ W_last + b_last

        # Store single-sample snapshot for the live visual neural network diagram
        if x.shape[0] == 1:
            self.last_layer_activations = [layer[0].copy() for layer in post_acts] + [
                np.tanh(z[0]).copy()
            ]

        return z, (pre_acts, post_acts)

    def act(self, obs, sigma, rng):
        z, cache = self.forward(obs)
        z0 = z[0]
        eps = rng.standard_normal(self.act_dim)
        action = np.clip(np.tanh(z0 + sigma * eps), -1.0, 1.0).astype(np.float32)
        if len(self.last_layer_activations) > 0:
            self.last_layer_activations[-1] = action.copy()
        return action, eps, cache

    def backward(self, dz, cache):
        """Backpropagate policy gradient through all 6 residual & LeakyReLU layers."""
        pre_acts, post_acts = cache
        p = self.params
        dz = np.atleast_2d(dz).astype(np.float64)
        grads = {}

        last = self.num_layers
        h_last = post_acts[-1]
        grads[f"W{last}"] = h_last.T @ dz
        grads[f"b{last}"] = dz.sum(axis=0)
        dh = dz @ p[f"W{last}"].T

        for i in range(last - 1, 0, -1):
            u = pre_acts[i - 1]
            h_prev = post_acts[i - 1]
            W = p[f"W{i}"]

            # Residual branch passes dh directly when dimensions match
            has_skip = h_prev.shape[1] == u.shape[1]
            du = dh * np.where(u > 0.0, 1.0, LEAKY_SLOPE)

            grads[f"W{i}"] = h_prev.T @ du
            grads[f"b{i}"] = du.sum(axis=0)

            dh_prev = du @ W.T
            dh = dh_prev + dh if has_skip else dh_prev

        return grads

    def num_params(self):
        return sum(v.size for v in self.params.values())

    def save(self, path):
        np.savez(
            path,
            obs_dim=self.obs_dim,
            hidden_dims=np.array(self.hidden_dims, dtype=np.int64),
            act_dim=self.act_dim,
            **self.params,
        )

    @classmethod
    def load(cls, path):
        with np.load(path) as data:
            obs_dim = int(data["obs_dim"])
            act_dim = int(data["act_dim"])
            if "hidden_dims" in data:
                hidden_dims = tuple(int(x) for x in data["hidden_dims"])
            else:
                hidden_dims = (int(data["hidden"]), int(data["hidden"]))
            net = cls(obs_dim=obs_dim, hidden_dims=hidden_dims, act_dim=act_dim)
            for key in net.params:
                if data[key].shape != net.params[key].shape:
                    raise ValueError(
                        f"{key} shape {data[key].shape} does not match {net.params[key].shape}"
                    )
                net.params[key] = data[key].astype(np.float64)
        return net


class Adam:
    def __init__(
        self, params, lr=7e-4, betas=(0.9, 0.999), eps=1e-8, max_grad_norm=2.5
    ):
        self.lr, self.betas, self.eps = lr, betas, eps
        self.max_grad_norm = max_grad_norm
        self.m = {k: np.zeros_like(v) for k, v in params.items()}
        self.v = {k: np.zeros_like(v) for k, v in params.items()}
        self.t = 0

    def step(self, params, grads):
        self.t += 1
        b1, b2 = self.betas

        # Global gradient norm clipping keeps deep network updates stable
        total_norm = np.sqrt(sum(np.sum(g * g) for g in grads.values()))
        clip_coef = min(1.0, self.max_grad_norm / (total_norm + 1e-8))

        for k in params:
            g = grads[k] * clip_coef
            self.m[k] = b1 * self.m[k] + (1 - b1) * g
            self.v[k] = b2 * self.v[k] + (1 - b2) * g * g
            m_hat = self.m[k] / (1 - b1**self.t)
            v_hat = self.v[k] / (1 - b2**self.t)
            params[k] -= self.lr * m_hat / (np.sqrt(v_hat) + self.eps)
