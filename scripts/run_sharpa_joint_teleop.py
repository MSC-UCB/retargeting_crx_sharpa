"""Run both Sharpa hands; preview by default, --backend ros enables ROS output."""

from retargeting_apps.sharpa_teleop import main


if __name__ == '__main__':
    raise SystemExit(main(with_arms=False))
