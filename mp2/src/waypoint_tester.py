from waypoint_list import WayPoints


def main():
    wp = WayPoints()
    wp.plotBezierPoints()

    print(wp.getWayPoints())


if __name__ == "__main__":
    main()