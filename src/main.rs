use clap::Parser;
use kurbo::{BezPath, PathEl};
use std::collections::HashMap;
use std::fs::File;
use std::io::{BufRead, BufReader};
use truck_modeling::*;

/// Simple program to generate a wing model given airfoil parameters
#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    /// Width of each wing
    #[arg(short = 'w', long)]
    semi_wingspan: f64,

    /// Distance to sweep the wing back
    #[arg(short, long, default_value_t = 0.0)]
    sweep: f64,

    /// Root chord length
    #[arg(short, long)]
    root_chord: f64,

    /// Tip chord length
    #[arg(short, long)]
    tip_chord: f64,

    /// Where to write the stl-formatted model
    #[arg(short, long)]
    outfile: String,

    /// Selig-formatted airfoil data
    file: String,
}

fn wire_from_path(path: BezPath, verts: &mut HashMap<(u64, u64), Vertex>) -> Wire {
    let mut vert = |p: kurbo::Point| {
        let (x, y) = (p.x, p.y);

        let k = (x.to_bits(), y.to_bits());
        if let Some(v) = verts.get(&k) {
            v.clone()
        } else {
            let v = builder::vertex(Point3::new(x, y, 0.0));
            verts.insert(k, v.clone());
            v
        }
    };

    let mut edges = Vec::with_capacity(path.elements().len());
    let mut last: Option<Vertex> = None;
    for seg in path.segments() {
        match seg {
            kurbo::PathSeg::Line(kurbo::Line { p0, p1 }) => {
                let end = vert(p1);
                edges.push(builder::line(&last.unwrap_or(vert(p0)), &end));
                last = Some(end);
            }
            kurbo::PathSeg::Quad(kurbo::QuadBez { p0, p1, p2 }) => {
                let end = vert(p2);
                edges.push(builder::bezier(
                    &last.unwrap_or(vert(p0)),
                    &end,
                    vec![Point3::new(p1.x, p1.y, 0.0)],
                ));
                last = Some(end);
            }
            kurbo::PathSeg::Cubic(kurbo::CubicBez { p0, p1, p2, p3 }) => {
                let end = vert(p3);
                edges.push(builder::bezier(
                    &last.unwrap_or(vert(p0)),
                    &end,
                    vec![Point3::new(p1.x, p1.y, 0.0), Point3::new(p2.x, p2.y, 0.0)],
                ));
                last = Some(end);
            }
        }
    }

    let (start, end) = (edges[0].front(), edges.last().unwrap().back());
    if start != end {
        edges.push(builder::line(end, start));
    }

    edges.into()
}

fn solid_to_stl(s: Solid, tolerance: f64) -> Vec<u8> {
    use truck_meshalgo::tessellation::MeshableShape;
    use truck_meshalgo::tessellation::MeshedShape;
    let mut mesh = s.triangulation(tolerance).to_polygon();

    use truck_meshalgo::filters::OptimizingFilter;
    mesh.put_together_same_attrs()
        .remove_degenerate_faces()
        .remove_unused_attrs();

    let mut out = Vec::with_capacity(1024);
    truck_polymesh::stl::write(&mesh, &mut out, truck_polymesh::stl::STLType::Binary).unwrap();

    out
}

fn extrude_then_transform<T: Sweep<Point3, Curve, Surface>>(
    elem: &T,
    extrude: Vector3,
    transform: Matrix4,
) -> T::Swept {
    let trsl_ex = Matrix4::from_translation(extrude);
    elem.sweep(
        &move |pt| transform.transform_point(trsl_ex.transform_point(*pt)),
        &move |curve| curve.transformed(trsl_ex).transformed(transform),
        &move |surface| surface.transformed(trsl_ex).transformed(transform),
        &move |pt0, pt1| Curve::Line(Line(*pt0, *pt1)),
        &move |curve0, curve1| match (curve0, curve1) {
            (Curve::Line(line), Curve::Line(_)) => Surface::Plane(Plane::new(
                line.0,
                line.1,
                transform.transform_point(line.1) + extrude,
            )),
            _ => unreachable!(),
        },
    )
}

fn main() {
    let args = Args::parse();
    let reader = BufReader::new(File::open(args.file).expect("failed opening file"));

    // Build a kurbo path from the airfoil data.
    let mut path = BezPath::from_vec(vec![
        PathEl::MoveTo((1., 0.).into()), // trailing edge
    ]);
    for (i, line) in reader.lines().enumerate() {
        if i < 2 {
            continue;
        }
        let line = line.unwrap();
        let line = line.trim_start().trim_end();
        if line.len() == 0 || line.starts_with("#") {
            continue;
        }

        let numbers: Vec<_> = line
            .split_terminator(&[' ', '\t'][..])
            .filter_map(|s| {
                if s == "" {
                    return None;
                };
                let f: Option<f64> = match s.parse() {
                    Ok(v) => Some(v),
                    Err(_) => None,
                };
                f
            })
            .collect();

        if numbers.len() != 2 {
            panic!(
                "expected 2 decimal datapoints per line, got {} on line {}",
                numbers.len(),
                i
            );
        }

        path.push(PathEl::LineTo((numbers[0], numbers[1]).into()));
    }

    let profile = builder::scaled(
        &wire_from_path(path, &mut HashMap::new()),
        Point3::new(0., 0., 0.),
        Vector3::new(args.root_chord, args.root_chord, args.root_chord),
    );
    let face: Face = builder::try_attach_plane(&vec![profile]).unwrap();
    let base: Shell = extrude_then_transform(
        &face,
        args.semi_wingspan * Vector3::unit_z(),
        Matrix4::from_nonuniform_scale(
            args.tip_chord / args.root_chord,
            args.tip_chord / args.root_chord,
            1.,
        ) + Matrix4::from_translation(args.sweep * Vector3::unit_x()),
    )
    .into_boundaries()
    .pop()
    .unwrap();

    let solid = Solid::new(vec![base]);
    let mut f = File::create(args.outfile).expect("Unable to create file");
    use std::io::Write;
    f.write(&solid_to_stl(solid, 0.05)).unwrap();
}
