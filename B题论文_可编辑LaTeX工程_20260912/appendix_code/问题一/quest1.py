import random

RADIUS = 1800
MIN_LIGHTS = 2
MAX_LIGHTS = 15


# 测试集生成
def random_point_in_circle(radius=RADIUS):
    while True:
        x = random.randint(-radius, radius)
        y = random.randint(-radius, radius)
        if x * x + y * y <= radius * radius:
            return x, y

def all_collinear(points):
    x0, y0 = points[0]
    for x1, y1 in points[1:]:
        if (x1, y1) != (x0, y0):
            break
    else:
        return True

    return all(
        (x1 - x0) * (y - y0) == (y1 - y0) * (x - x0)
        for x, y in points[1:]
    )

def generate_points():
    while True:
        light_count = random.randint(MIN_LIGHTS, MAX_LIGHTS)
        points = set()

        while len(points) < light_count + 1:
            points.add(random_point_in_circle())

        points = list(points)
        if not all_collinear(points):
            return points[:light_count], points[-1]


        
# -------------------------------------------------------
# 探测目标
import math

BEAM_WIDTH = 2.0
ROTATION_STEP = 2.0
ANGLE_EPSILON = 1e-10


def normalize_angle(angle):
    return angle % 360.0


def target_direction(searchlight, target_point):
    dx = target_point[0] - searchlight[0]
    dy = target_point[1] - searchlight[1]
    return normalize_angle(math.degrees(math.atan2(dy, dx)))


def target_in_beam(searchlight, target_point, center_angle):
    """Return True when the target is inside or on the 2-degree beam."""
    direction = target_direction(searchlight, target_point)
    angle_difference = (direction - center_angle + 180.0) % 360.0 - 180.0
    return abs(angle_difference) <= BEAM_WIDTH / 2.0 + ANGLE_EPSILON


def direction_vector(angle):
    radians = math.radians(angle)
    return math.cos(radians), math.sin(radians)


def generate_and_detect_beam(searchlight, target_point):
    """Rotate one beam counterclockwise until it first detects the target."""
    initial_angle = random.uniform(0.0, 360.0)


    for rotation_count in range(round(360.0 / ROTATION_STEP)):
        center_angle = normalize_angle(
            initial_angle + rotation_count * ROTATION_STEP
        )
        if target_in_beam(searchlight, target_point, center_angle):
            left_angle = normalize_angle(center_angle - BEAM_WIDTH / 2.0)
            right_angle = normalize_angle(center_angle + BEAM_WIDTH / 2.0)
            return {
                "initial_angle": initial_angle,
                "center_angle": center_angle,
                "rotation_count": rotation_count,
                "left_angle": left_angle,
                "right_angle": right_angle,
                "left_vector": direction_vector(left_angle),
                "right_vector": direction_vector(right_angle),
            }

    raise RuntimeError("The beam failed to detect the target in one rotation.")


def detect_with_all_searchlights(searchlight_points, target_point):
    return [
        generate_and_detect_beam(searchlight, target_point)
        for searchlight in searchlight_points
    ]



# -------------------------------------------------------
# 半平面交求多边形
GEOMETRY_EPSILON = 1e-8

def beam_to_halfplanes(searchlight, beam_result):
    """Convert the two beam boundaries to half-planes containing the target."""
    sx, sy = searchlight
    lower_x, lower_y = beam_result["left_vector"]
    upper_x, upper_y = beam_result["right_vector"]

    lower_halfplane = (
        -lower_y,
        lower_x,
        lower_y * sx - lower_x * sy,
    )
    upper_halfplane = (
        upper_y,
        -upper_x,
        -upper_y * sx + upper_x * sy,
    )
    return lower_halfplane, upper_halfplane


def line_intersection(first, second):
    a1, b1, c1 = first
    a2, b2, c2 = second
    determinant = a1 * b2 - a2 * b1
    if abs(determinant) <= GEOMETRY_EPSILON:
        return None

    x = (b1 * c2 - b2 * c1) / determinant
    y = (c1 * a2 - a1 * c2) / determinant
    return x, y


def inside_all_halfplanes(point, halfplanes):
    x, y = point
    return all(
        a * x + b * y + c >= -GEOMETRY_EPSILON
        for a, b, c in halfplanes
    )


def intersection_is_unbounded(halfplanes):
    """Check whether the common recession cone has a nonzero direction."""
    for a, b, _ in halfplanes:
        for dx, dy in ((b, -a), (-b, a)):
            if all(
                other_a * dx + other_b * dy >= -GEOMETRY_EPSILON
                for other_a, other_b, _ in halfplanes
            ):
                return True
    return False


def halfplane_intersection(halfplanes):
    """Find all vertices by intersecting boundary pairs and testing feasibility."""
    vertices = []
    for first_index in range(len(halfplanes)):
        for second_index in range(first_index + 1, len(halfplanes)):
            point = line_intersection(
                halfplanes[first_index], halfplanes[second_index]
            )
            if point is None or not inside_all_halfplanes(point, halfplanes):
                continue
            if not any(
                (point[0] - old_point[0]) ** 2
                + (point[1] - old_point[1]) ** 2
                <= GEOMETRY_EPSILON ** 2
                for old_point in vertices
            ):
                vertices.append(point)

    if len(vertices) < 3 or intersection_is_unbounded(halfplanes):
        return []

    center_x = sum(point[0] for point in vertices) / len(vertices)
    center_y = sum(point[1] for point in vertices) / len(vertices)
    vertices.sort(
        key=lambda point: math.atan2(point[1] - center_y, point[0] - center_x)
    )
    return vertices


def generate_bounded_result():
    """Regenerate the complete sample until the beam intersection is bounded."""
    while True:
        searchlight_points, target_point = generate_points()
        results = detect_with_all_searchlights(searchlight_points, target_point)
        halfplanes = [
            halfplane
            for searchlight, result in zip(searchlight_points, results)
            for halfplane in beam_to_halfplanes(searchlight, result)
        ]
        vertices = halfplane_intersection(halfplanes)
        if vertices:
            return searchlight_points, target_point, results, vertices


if __name__ == "__main__":
    searchlights, target, beam_results, polygon_vertices = generate_bounded_result()

    light_and_vectors = [
        (
            searchlight,
            tuple(round(value, 6) for value in result["left_vector"]),
            tuple(round(value, 6) for value in result["right_vector"]),
        )
        for searchlight, result in zip(searchlights, beam_results)
    ]
    rounded_vertices = [
        tuple(round(value, 6) for value in vertex)
        for vertex in polygon_vertices
    ]

    print("探照灯坐标：", searchlights)
    print("目标点坐标：", target)
    print("探照灯及对应边界向量：", light_and_vectors)
    print("多边形顶点坐标：", rounded_vertices)


# -------------------------------------------------------
# 旋转卡壳求直径
def triangle_double_area(first, second, third):
    return abs(
        (second[0] - first[0]) * (third[1] - first[1])
        - (second[1] - first[1]) * (third[0] - first[0])
    )


def squared_distance(first, second):
    return (first[0] - second[0]) ** 2 + (first[1] - second[1]) ** 2


def rotating_calipers_diameter(vertices):
    vertex_count = len(vertices)
    if vertex_count < 2:
        return 0.0
    if vertex_count == 2:
        return math.sqrt(squared_distance(vertices[0], vertices[1]))

    opposite_index = 1
    maximum_distance_squared = 0.0

    for index in range(vertex_count):
        next_index = (index + 1) % vertex_count

        while triangle_double_area(
            vertices[index],
            vertices[next_index],
            vertices[(opposite_index + 1) % vertex_count],
        ) > triangle_double_area(
            vertices[index],
            vertices[next_index],
            vertices[opposite_index],
        ) + GEOMETRY_EPSILON:
            opposite_index = (opposite_index + 1) % vertex_count

        maximum_distance_squared = max(
            maximum_distance_squared,
            squared_distance(vertices[index], vertices[opposite_index]),
            squared_distance(vertices[next_index], vertices[opposite_index]),
        )

    return math.sqrt(maximum_distance_squared)


# -------------------------------------------------------
# 覆盖判断

def find_diameter_segment(vertices, diameter_value):
    """Find the vertex pair corresponding to the calculated diameter."""
    diameter_squared = diameter_value * diameter_value
    tolerance = GEOMETRY_EPSILON * max(1.0, diameter_squared)

    for first_index in range(len(vertices)):
        for second_index in range(first_index + 1, len(vertices)):
            if abs(
                squared_distance(vertices[first_index], vertices[second_index])
                - diameter_squared
            ) <= tolerance:
                return vertices[first_index], vertices[second_index]

    raise RuntimeError("No vertex pair corresponding to the diameter was found.")


def diameter_circle_covers_polygon(vertices, diameter_value):
    """A convex polygon is covered iff all its vertices are inside the disk."""
    first_endpoint, second_endpoint = find_diameter_segment(
        vertices, diameter_value
    )
    center = (
        (first_endpoint[0] + second_endpoint[0]) / 2.0,
        (first_endpoint[1] + second_endpoint[1]) / 2.0,
    )
    radius_squared = diameter_value * diameter_value / 4.0
    tolerance = GEOMETRY_EPSILON * max(1.0, radius_squared)

    return all(
        squared_distance(vertex, center) <= radius_squared + tolerance
        for vertex in vertices
    )






if __name__ == "__main__":
    diameter = rotating_calipers_diameter(polygon_vertices)
    print("多边形直径：", round(diameter, 6))

    if diameter_circle_covers_polygon(polygon_vertices, diameter):
            print("以定位区域直径为直径的圆能覆盖此定位区域")
    else:
            print("以定位区域直径为直径的圆不能覆盖此定位区域")
    
