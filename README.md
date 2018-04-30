# cgmath frustum culling

[![Build Status](https://travis-ci.org/germangb/cgmath-culling.svg?branch=master)](https://travis-ci.org/germangb/cgmath-culling)

Small [frustum culling](https://en.wikipedia.org/wiki/Hidden_surface_determination#Viewing_frustum_culling) crate meant to be used alongside the `cgmath` crate.

## Usage

```rust
extern crate cgmath;
extern crate cgmath_culling;

use cgmath::{PerspectiveFov, Rad};
use cgmath_culling::{FrustumCuller, Intersection};

let per = PerspectiveFov { fovy: Rad(3.1415_f32 / 2.0),
                                  aspect: 1.0,
                                  near: 0.1,
                                  far: 100.0 }.into();

let culling = FrustumCuller::from_perspective_fov(per);

match culling.intersect_aab(Vector3::new(0.0, 0.0, -7.0), Vector3::new(1.0, 1.0, -5.0)) {
    Intersection::Inside  => println!("I'm inside"),
    Intersection::Outside => println!("I'm outside"),
    Intersection::Partial => println!("I'm partially inside"),
}
```

## License

[MIT](LICENSE.md)

## Disclaimer

* Reference implementation: [JOML](https://github.com/JOML-CI/JOML)
