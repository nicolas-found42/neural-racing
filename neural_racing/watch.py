"""Watch a trained policy drive."""

import os

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

import argparse

import numpy as np
import pygame

from .game import DECISION_DT, Env, build_screen, draw_scene
from .network import Policy

FPS = 20


def main():
    parser = argparse.ArgumentParser(description="Watch the trained car drive a lap.")
    parser.add_argument("--weights", default="weights.npz")
    parser.add_argument("--episodes", type=int, default=0, help="number of runs, 0 means forever")
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    policy = Policy.load(args.weights)
    env = Env(seed=args.seed)
    screen = build_screen("Watch mode")
    font = pygame.font.SysFont("menlo", 14)
    clock = pygame.time.Clock()
    rng = np.random.default_rng(args.seed)

    episode = 0
    lap_time = 0.0
    best = 0.0
    obs = env.reset()
    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                running = False

        # One step is already three physics substeps, so the policy decides here.
        action, _, _ = policy.act(obs, 0.0, rng)
        obs, reward, done, info = env.step(action)
        lap_time += DECISION_DT
        best = max(best, info["distance"])

        draw_scene(screen, env, font, [
            f"speed {env.speed:5.0f} px/s",
            f"distance {info['distance']:6.0f} / {env.track.LAP_LENGTH:.0f} px",
            f"lap {1.0 if info['lap_done'] else 0.0:.0f}   t {lap_time:4.1f}s   best {best:6.0f}",
        ])
        pygame.display.flip()
        clock.tick(FPS)

        if done:
            episode += 1
            print(f"episode {episode}: distance {info['distance']:.0f} px, "
                  f"lap {'yes' if info['lap_done'] else 'no'}, "
                  f"off track {'yes' if info['off_track'] else 'no'}, {lap_time:.2f}s")
            if args.episodes and episode >= args.episodes:
                running = False
            obs = env.reset()
            lap_time = 0.0

    pygame.quit()


if __name__ == "__main__":
    main()
