import math

class Formation:
    def __init__(self, center_x=0, center_y=0):
        self.center_x = center_x
        self.center_y = center_y

    def distribute(self, k, n):
        raise NotImplementedError("This method should be implemented by subclasses.")


class Circle(Formation):
    def __init__(self, center_x=0, center_y=0, radius=1, **kwargs):
        super().__init__(center_x, center_y)
        self.radius = radius

    def distribute(self, k, n):
        x = self.radius * math.cos(k / n * 2 * math.pi) + self.center_x
        y = self.radius * math.sin(k / n * 2 * math.pi) + self.center_y
        return x, y


class Line(Formation):
    def __init__(self, center_x=0, center_y=0, separation=0.5, direction='horizontal', **kwargs):
        super().__init__(center_x, center_y)
        self.separation = separation
        self.direction = direction

    def distribute(self, k, n):
        if self.direction == 'horizontal':
            x = self.center_x + (k - (n - 1) // 2) * self.separation
            y = self.center_y
        else:  # vertical
            x = self.center_x
            y = self.center_y + (k - (n - 1) // 2) * self.separation
        return x, y


class TwoLines(Formation):
    def __init__(self, center_x=0, center_y=0, separation=1, direction='horizontal', **kwargs):
        super().__init__(center_x, center_y)
        self.separation = separation
        self.direction = direction

    def distribute(self, k, n):
        half_n = n // 2
        line = 0
        if n % 2 != 0:
            if k < half_n + 1:
                line = 0
            else:
                line = 1
                k -= half_n + 1
        else:
            if k >= half_n:
                line = 1
                k -= half_n

        if self.direction == 'horizontal':
            x = self.center_x + k * self.separation
            y = self.center_y - line * self.separation
        else:  # vertical
            x = self.center_x - line * self.separation
            y = self.center_y + k * self.separation
        return x, y


class ThreeLines(Formation):
    def __init__(self, center_x=0, center_y=0, separation=0.5, direction='horizontal', **kwargs):
        super().__init__(center_x, center_y)
        self.separation = separation
        self.direction = direction

    def distribute(self, k, n) -> tuple:
        """Distributes robots in three lines. The robots are distributed as evenly as possible across the three lines."""
        # Determine the number of robots in each line
        robots_per_line = n // 3
        extra_robots = n % 3

        # Assign extra robots to the first and second line as needed
        line_robots = [robots_per_line] * 3
        for i in range(extra_robots):
            line_robots[i] += 1

        # Determine the line number and adjust k accordingly
        if k < line_robots[0]:
            line = 0
        elif k < line_robots[0] + line_robots[1]:
            line = 1
            k -= line_robots[0]
        else:
            line = 2
            k -= (line_robots[0] + line_robots[1])

        # Calculate the position based on the line and direction
        if self.direction == 'horizontal':
            x = self.center_x + k * self.separation
            y = self.center_y - line * self.separation
        else:  # vertical
            x = self.center_x - line * self.separation
            y = self.center_y + k * self.separation

        return x, y