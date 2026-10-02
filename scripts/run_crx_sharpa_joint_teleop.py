"""Run CRX arms and Sharpa hands with ROS output and the dual-hand stop gesture."""

from retargeting_apps.sharpa_teleop import main


if __name__ == '__main__':
    raise SystemExit(main(with_arms=True))
