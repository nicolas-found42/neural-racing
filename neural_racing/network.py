"""Feedforward policy and Adam, in plain numpy."""

import numpy as np


class Policy:
    def __init__(self, obs_dim=13, hidden=64, act_dim=2, seed=0):
        rng = np.random.default_rng(seed)
        self.obs_dim, self.hidden, self.act_dim = obs_dim, hidden, act_dim
        self.params = {
            "W1": rng.standard_normal((obs_dim, hidden)) * np.sqrt(2.0 / obs_dim),
            "b1": np.zeros(hidden),
            "W2": rng.standard_normal((hidden, hidden)) * np.sqrt(2.0 / hidden),
            "b2": np.zeros(hidden),
            "W3": rng.uniform(-0.01, 0.01, (hidden, act_dim)),
            "b3": np.zeros(act_dim),
        }
        # Start with a mild throttle bias. A zero-mean random throttle decays
        # speed under drag, so the untrained car rolls backward out of the
        # corridor and every episode earns nothing to reinforce.
        self.params["b3"][1] = np.arctanh(0.5)

    def forward(self, obs):
        p = self.params
        obs = np.atleast_2d(obs)
        h1 = np.maximum(0.0, obs @ p["W1"] + p["b1"])
        h2 = np.maximum(0.0, h1 @ p["W2"] + p["b2"])
        z = h2 @ p["W3"] + p["b3"]
        return z, (obs, h1, h2)

    def act(self, obs, sigma, rng):
        z, cache = self.forward(obs)
        z = z[0]
        eps = rng.standard_normal(self.act_dim)
        action = np.clip(np.tanh(z + sigma * eps), -1.0, 1.0).astype(np.float32)
        return action, eps, cache

    def backward(self, dz, cache):
        """d(dz dot z)/d(params). dz may be (act_dim,) or (batch, act_dim)."""
        obs, h1, h2 = cache
        p = self.params
        dz = np.atleast_2d(dz)
        g = {}
        g["W3"] = h2.T @ dz
        g["b3"] = dz.sum(axis=0)
        dh2 = (dz @ p["W3"].T) * (h2 > 0)
        g["W2"] = h1.T @ dh2
        g["b2"] = dh2.sum(axis=0)
        dh1 = (dh2 @ p["W2"].T) * (h1 > 0)
        g["W1"] = obs.T @ dh1
        g["b1"] = dh1.sum(axis=0)
        return g

    def num_params(self):
        return sum(v.size for v in self.params.values())

    def save(self, path):
        np.savez(path, obs_dim=self.obs_dim, hidden=self.hidden, act_dim=self.act_dim, **self.params)

    @classmethod
    def load(cls, path):
        with np.load(path) as data:
            net = cls(int(data["obs_dim"]), int(data["hidden"]), int(data["act_dim"]))
            for key in net.params:
                if data[key].shape != net.params[key].shape:
                    raise ValueError(f"{key} shape {data[key].shape} does not match {net.params[key].shape}")
                net.params[key] = data[key].astype(np.float64)
        return net


class Adam:
    def __init__(self, params, lr=1e-3, betas=(0.9, 0.999), eps=1e-8):
        self.lr, self.betas, self.eps = lr, betas, eps
        self.m = {k: np.zeros_like(v) for k, v in params.items()}
        self.v = {k: np.zeros_like(v) for k, v in params.items()}
        self.t = 0

    def step(self, params, grads):
        self.t += 1
        b1, b2 = self.betas
        for k in params:
            g = grads[k]
            self.m[k] = b1 * self.m[k] + (1 - b1) * g
            self.v[k] = b2 * self.v[k] + (1 - b2) * g * g
            m_hat = self.m[k] / (1 - b1 ** self.t)
            v_hat = self.v[k] / (1 - b2 ** self.t)
            params[k] -= self.lr * m_hat / (np.sqrt(v_hat) + self.eps)
