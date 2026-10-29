# Neural Racing

A car learns to drive a top-down oval track with reinforcement learning. The policy is a small feedforward network written in numpy, trained with REINFORCE on a reward that pays for progress along the track and punishes wall contact and leaving the road.

pygame draws the world. There are no ML dependencies.

## Install

```sh
python3.13 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

Python 3.13 is what the project was tested on. pygame has no wheels for 3.15 yet, so a venv on 3.13 is the safe choice.

## Train

```sh
.venv/bin/python -m neural_racing.train                 # 800 episodes, headless, saves weights.npz
.venv/bin/python -m neural_racing.train --render        # same, with the window open
.venv/bin/python -m neural_racing.train --episodes 3000 --out run2.npz --seed 3
```

Progress prints every ten episodes. The saved file is the best episode so far, measured by distance travelled. Training is headless by default so it runs at full speed.

The default run of 800 episodes takes about 36 seconds on an M-series laptop. Distance per episode climbs from under 100 px in the first tens of episodes to a full 1742 px lap, and the run above ended with 228 laps in its last 600 episodes.

## Watch

```sh
.venv/bin/python -m neural_racing.watch
.venv/bin/python -m neural_racing.watch --weights run2.npz
.venv/bin/python -m neural_racing.watch --episodes 3
```

Watch mode runs the network deterministically (no exploration noise) at 20 decisions per second, draws the car, and draws the eight ray casts that feed the network. Each finished episode prints its distance and whether it was a clean lap. Press Escape or close the window to quit.

## How it works

The car has a position, a heading and a scalar speed along that heading. Each decision applies steering and throttle for three 1/60 second physics steps, so the agent acts 20 times per second, matching the 30 second episodes in the source video.

| Piece | Shape | Meaning |
|---|---|---|
| Observation | 13 | normalized position, normalized speed, heading as sine and cosine, then 8 wall distances |
| Action | 2 | steering in [-1, 1], throttle in [-1, 1] where negative brakes and reverses |
| Policy | 13, 64, 64, 2 | ReLU on both hidden layers, tanh on the output |

Rays fan out over 180 degrees ahead of the car and stop at the first boundary hit, capped at 400 px. The track is a stadium whose centerline is a closed polyline. The two boundaries are that polyline offset by plus and minus 80 px, so the corridor is 160 px wide and the car starts on the racing line at the beginning of the bottom straight.

The reward each decision is the forward progress along the track tangent, minus a small time cost, minus a wall proximity penalty that grows as the nearest ray drops below 80 px, minus 10 and the episode ends when the car leaves the corridor, plus 10 when it crosses back over the start line having covered a full lap.

Training collects four episodes, computes discounted returns with a 0.97 factor, normalizes advantages across the whole batch, and takes one Adam step at 1e-3. Exploration is a Gaussian in the space before the tanh output, and its standard deviation falls from 0.4 toward 0.08 over 2000 episodes. Because the noise is added before the tanh, the policy gradient needs no tanh derivative.

Two details carry most of the result. The untrained policy starts with a mild throttle bias, because a zero-mean random throttle decays speed under drag and the car rolls backward out of the corridor, leaving nothing to reinforce. And advantages are normalized across the batch rather than per episode, so a clean run and a crash are graded against each other.

Across eight seeds at 700 episodes, the greedy saved policy completed a lap in five.

## Source layout

- `neural_racing/game.py` defines the track, physics, ray casting, reward, and episode rules.
- `neural_racing/network.py` defines the policy and Adam optimizer.
- `neural_racing/train.py` runs training and saves the best weights.
- `neural_racing/watch.py` loads weights and runs the visual demo.
- `weights.npz` stores the network parameters and layer sizes.
