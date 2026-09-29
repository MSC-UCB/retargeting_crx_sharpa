"""Run both CRX arms and Sharpa hands; select --backend ros for ROS output."""

from retargeting_apps.sharpa_teleop import main


if __name__ == '__main__':
    raise SystemExit(main(with_arms=True))
