use simulation::body::Body;
use point::Point;
use simulation::shape::Shape;
use simulation::polygon::Polygon;
use simulation::circle::Circle;
use simulation::collisions::CollisionInfo;

pub fn find_collisions(body1: &Body, body2: &Body) -> Vec<CollisionInfo> {
    let mut collisions = vec![];
    match body1.shape {
        Shape::Circle(ref circle1) => {
            match body2.shape {
                Shape::Circle(ref circle2) => { 
                    collisions.push(circle_circle(&circle1, &circle2));
                }
                _ => {}
            }
        }
        Shape::Polygon(ref polygon1) => {
            match body2.shape {
                Shape::Polygon(ref polygon2) => { 
                    let colls = polygon_polygon(&polygon1, &polygon2);
                    collisions.push(colls.0);
                    collisions.push(colls.1);
                }
                _ => {}
            }
        }
    }
    collisions.iter().filter_map(|&x| x).collect()
}

// fn circle_circle(circle1: &Circle, circle2: &Circle) -> Option<Collision> {
//     let normal = circle1.pos - circle2.pos;
//     let distance = normal.norm();
//     let depth = distance - (circle1.radius + circle2.radius);
//     if depth < 0.0 {
//         Some(Collision { 
//             pos: circle1.pos.middle(circle2.pos),
//             depth: depth,
//             normal: normal
//         })
//     }
//     else {
//         None
//     }
// }

fn circle_circle(circle1: &Circle, circle2: &Circle) -> Option<CollisionInfo> {
    None
}

fn polygon_polygon(polygon1: &Polygon, polygon2: &Polygon) -> (Option<CollisionInfo>, Option<CollisionInfo>) {
    let mut min_depth_normal: Option<(f64, Point)> = None;
    for (i, &edge) in polygon1.get_normals().iter().chain(polygon2.get_normals().iter()).enumerate() {
        let projection1 = polygon1.project(edge);
        let projection2 = polygon2.project(edge);
        let depth = get_depth_from_projections(projection1, projection2);
        if depth < 0.0 {
            return (None, None)
        }
        else {
            let is_deeper = match min_depth_normal {
                None => { true }
                Some((min_depth, _)) => { depth < min_depth }
            };
            if is_deeper {
                let normal = edge;
                let corrected_normal = match (polygon1.pos - polygon2.pos) * normal < 0.0 {
                    true => { normal.clone() }
                    false => { -normal.clone() }
                };
                min_depth_normal = Some((depth, corrected_normal));
            }
        }
    }
    let collision_pos = get_collision_pos(polygon1, polygon2, min_depth_normal.unwrap().1);
    let create_collision_info = |pos| CollisionInfo {
        depth: min_depth_normal.unwrap().0,
        normal: min_depth_normal.unwrap().1,
        pos: pos
    };
    (Some(create_collision_info(collision_pos.0)), collision_pos.1.map(create_collision_info))
}

fn get_closest_points(polygon1: &Polygon, polygon2: &Polygon, normal: Point) -> (Point, Option<Point>){
    // Find the point of polygon1 that is closest to polygon2 (along the normal).
    // Also return the second closest point if the distance of the second closest 
    // point is almost as small as the closest point
    let mut closest: Option<(Point, f64)> = None;
    let mut second_closest: Option<(Point, f64)> = None;
    let comparison = polygon2.pos * normal;
    for &point in polygon1.vertices.iter() {
        let distance = (point * normal - comparison).abs();
        match closest {
            None => { 
                closest = Some((point, distance)); continue 
            },
            Some((_closest_point, closest_distance)) => {
                if distance + super::COLLISION_MARGIN < closest_distance {
                    second_closest = None;
                    closest = Some((point, distance));
                }
                else if distance < closest_distance {
                    second_closest = closest;
                    closest = Some((point, distance));
                }
                else if distance < closest_distance + super::COLLISION_MARGIN {
                    second_closest = Some((point, distance));
                }
            }
        }

    }
    (closest.unwrap().0, second_closest.map(|x| x.0))
}

fn get_collision_pos(polygon1: &Polygon, polygon2: &Polygon, normal: Point) -> (Point, Option<Point>) {
    let (closest1, second_closest_1) = get_closest_points(polygon1, polygon2, normal);
    let (closest2, second_closest_2) = get_closest_points(polygon2, polygon1, normal);
    match second_closest_1 {
        None => {
            match second_closest_2 {
                // Point-Point: Return the middle
                None => ((closest1 + closest2) * 0.5, None),
                // Point-Line: Return the point
                Some(_) => (closest1, None)
            }
        },
        Some(second_closest_point_1) => {
            match second_closest_2 {
                // Line-Point: Return the point
                None => (closest2, None),
                // Line-Line: Find the proper manifold between the four points.
                Some(second_closest_point_2) => {
                    let (point1, point2) = find_manifold(closest1, second_closest_point_1, closest2, second_closest_point_2, normal);
                    (point1, Some(point2))

                }
            }
        }
    }
}

fn find_manifold(p1: Point, q1: Point, p2: Point, q2: Point, normal: Point) -> (Point, Point) {
    // Determine the contact line between the four points that make up two touching
    // lines.
    // Four cases to consider
    // 1)
    // p1 |        | q1
    // p2    |  |    q2
    //       ____
    // 2)
    // p1    |     | q1
    // p2 |     |    q2
    //       ____
    // 3)
    // p1 |     |    q1
    // p2    |     | q2
    //       ____
    // 4)
    // p1    |  |    q1
    // p2 |        | q2
    //       ____
    let line = normal.orth();
    let p1_projected = p1 * line;
    let q1_projected = q1 * line;
    let p2_projected = p2 * line;
    let q2_projected = q2 * line;
    let min1 = p1_projected.min(q1_projected);
    let min2 = p2_projected.min(q2_projected);
    let max1 = p1_projected.max(q1_projected);
    let max2 = p2_projected.max(q2_projected);
    let start_manifold_projected = min1.max(min2);
    let end_manifold_projected = max1.min(max2);
    // All of these points lie on a line (approximately). This line is parametrized by 
    // p + t * line where p is any of the four points. Therefore we may choose any of them.
    // Here we choose p1.
    let start_manifold = p1 + line * (start_manifold_projected - p1_projected);
    let end_manifold = p1 + line * (end_manifold_projected - p1_projected);
    (start_manifold, end_manifold)
}

fn get_depth_from_projections(projection1: [f64; 2], projection2: [f64; 2]) -> f64 {
    return (projection1[1] - projection2[0]).min(projection2[1] - projection1[0]);
}

#[cfg(test)]
mod tests{
    use super::*;
    #[test] 
    fn test_closest_points() {
        let mut vertices = vec![];
        vertices.push(Point::new(0.0, 0.0));
        vertices.push(Point::new(1.0, 0.0));
        vertices.push(Point::new(1.0, 1.0));
        vertices.push(Point::new(0.0, 1.0));
        let poly1 = Polygon::new(vertices);
        vertices = vec![];
        vertices.push(Point::new(2.0, 0.0));
        vertices.push(Point::new(3.0, 0.0));
        vertices.push(Point::new(3.0, 1.0));
        vertices.push(Point::new(2.0, 1.0));
        let poly2 = Polygon::new(vertices);
        let points = get_closest_points(&poly1, &poly2, Point::new(1.0, 0.0));
        let is_close = |p1: Point, p2: Point| (p1-p2).norm() < 0.0001;
        assert!(is_close(points.0, Point::new(1.0, 0.0)));
        assert!(is_close(points.1.unwrap(), Point::new(1.0, 1.0)));

        vertices = vec![];
        vertices.push(Point::new(0.0, 0.0));
        vertices.push(Point::new(1.5, 0.0));
        vertices.push(Point::new(1.0, 1.0));
        vertices.push(Point::new(0.0, 1.0));
        let poly1 = Polygon::new(vertices);
        let points = get_closest_points(&poly1, &poly2, Point::new(1.0, 0.0));
        let is_close = |p1: Point, p2: Point| (p1-p2).norm() < 0.0001;
        assert!(is_close(points.0, Point::new(1.5, 0.0)));
        assert!(match points.1 { None => true, _ => false });
    }
}
