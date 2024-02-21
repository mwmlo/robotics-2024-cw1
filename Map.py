# A Map class containing walls
from Particle import Particle


class Map:

    def __init__(self, visualise):
        self.walls = []
        self.areas = []
        self.visualise = visualise

    def add_wall(self, wall):
        self.walls.append(wall)

    def add_area(self, x0, x1, y0, y1):
        self.areas.append((x0, x1, y0, y1))

    def clear(self):
        self.walls = []

    def draw(self):
        for wall in self.walls:
            self.visualise.draw_line(wall)

    def is_particle_in(self, particle: Particle):
        for area in self.areas:
            if (min(area[0], area[1]) <= particle.x <= max(area[0], area[1])) and (
                    min(area[2], area[3]) <= particle.y <= max(area[2], area[3])):
                return True

        return False


def myMap(visualise):
    mymap = Map(visualise)
    # Definitions of walls
    # a: O to A
    # b: A to B
    # c: C to D
    # d: D to E
    # e: E to F
    mymap.add_wall((0, 0, 0, 168))  # a
    mymap.add_wall((0, 168, 84, 168))  # b
    mymap.add_wall((84, 126, 84, 210))  # c
    mymap.add_wall((84, 210, 168, 210))  # d
    # f: F to G
    # g: G to H
    # h: H to O
    mymap.add_wall((0, 0, 0, 168))  # a
    mymap.add_wall((0, 168, 84, 168))  # b
    mymap.add_wall((84, 126, 84, 210))  # c
    mymap.add_wall((84, 210, 168, 210))  # d
    mymap.add_wall((168, 210, 168, 84))  # e
    mymap.add_wall((168, 84, 210, 84))  # f
    mymap.add_wall((210, 84, 210, 0))  # g
    mymap.add_wall((210, 0, 0, 0))  # h
    mymap.draw()
    return mymap
