# cgmath frustum culling

[![Build Status](https://travis-ci.org/germangb/cgmath-culling.svg?branch=master)](https://travis-ci.org/germangb/cgmath-culling)

## Usage

```rust
extern crate cgmath;
extern crate cgmath_culling;

use cgmath::{PerspectiveFov, Rad};
use cgmath_culling::{FrustumCuller, Intersection};

// Projection matrix
let projection: Matrix4<f32> = PerspectiveFov { fovy: Rad(3.14159265 / 2.0),aspect: 1.0, near: 0.1, far: 100.0 }.into();

let culling = FrustumCuller::from_matrix(projection);

match culling.intersect_aab(Vector3::new(0.0, 0.0, -7.0), Vector3::new(1.0, 1.0, -5.0)) {
    Intersection::Inside  => println!("I'm inside"),
    Intersection::Outside => println!("I'm outside"),
    Intersection::Partial => println!("I'm partially inside"),
}
```

## License

```
The MIT License (MIT)

Copyright (c) 2018 German Gomez Bajo

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
```

## Disclaimer

* Reference implementation: [JOML](https://github.com/JOML-CI/JOML)
