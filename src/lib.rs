extern crate cgmath;

use std::mem;

use cgmath::{BaseFloat, Matrix4, Vector3, Vector4, prelude::*};

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct FrustumIntersection<S> {
    nx_x: S,
    nx_y: S,
    nx_z: S,
    nx_w: S,
    px_x: S,
    px_y: S,
    px_z: S,
    px_w: S,
    ny_x: S,
    ny_y: S,
    ny_z: S,
    ny_w: S,
    py_x: S,
    py_y: S,
    py_z: S,
    py_w: S,
    nz_x: S,
    nz_y: S,
    nz_z: S,
    nz_w: S,
    pz_x: S,
    pz_y: S,
    pz_z: S,
    pz_w: S,
}

#[derive(Debug, PartialEq)]
pub enum IntersectionResult {
    Inside,
    Outside,
    Intersect,
}

impl<S: BaseFloat> FrustumIntersection<S> {
    /// Create a FrustumIntersection given a matrix from with the frustum planes are extracted.
    ///
    /// In order to update the frustum later on, use the update method.
    pub fn from_matrix4(m: Matrix4<S>) -> Self {
        let mut intersect: Self = unsafe { mem::zeroed() };
        intersect.update(m);
        intersect
    }

    /// Updates the frustum, using a matrix, from which the frustum planes are extracted.
    ///
    /// If this `FrustumIntersection` is meant to be used with spheres, use the method
    /// `update_with_spheres` instead.
    #[inline]
    pub fn update(&mut self, m: Matrix4<S>) {
        self.update_with_spheres(m, true);
    }

    /// Updates the frustum, using a matrix, from which the frustum planes are extracted.
    ///
    /// If this FrustumIntersection will be used to test spheres, you can indicate so with
    /// the `allow_test_spheres` parameter
    pub fn update_with_spheres(&mut self, m: Matrix4<S>, allow_test_spheres: bool) {
        self.nx_x = m.x.w + m.x.x;
        self.nx_y = m.y.w + m.y.x;
        self.nx_z = m.z.w + m.z.x;
        self.nx_w = m.w.w + m.w.x;
        if (allow_test_spheres) {
            let invl = (self.nx_x * self.nx_x + self.nx_y * self.nx_y + self.nx_z * self.nx_z)
                .sqrt()
                .recip();
            self.nx_x *= invl;
            self.nx_y *= invl;
            self.nx_z *= invl;
            self.nx_w *= invl;
        }
        self.px_x = m.x.w - m.x.x;
        self.px_y = m.y.w - m.y.x;
        self.px_z = m.z.w - m.z.x;
        self.px_w = m.w.w - m.w.x;
        if (allow_test_spheres) {
            let invl = (self.px_x * self.px_x + self.px_y * self.px_y + self.px_z * self.px_z)
                .sqrt()
                .recip();
            self.px_x *= invl;
            self.px_y *= invl;
            self.px_z *= invl;
            self.px_w *= invl;
        }
        self.ny_x = m.x.w + m.x.y;
        self.ny_y = m.y.w + m.y.y;
        self.ny_z = m.z.w + m.z.y;
        self.ny_w = m.w.w + m.w.y;
        if (allow_test_spheres) {
            let invl = (self.ny_x * self.ny_x + self.ny_y * self.ny_y + self.ny_z * self.ny_z)
                .sqrt()
                .recip();
            self.ny_x *= invl;
            self.ny_y *= invl;
            self.ny_z *= invl;
            self.ny_w *= invl;
        }
        self.py_x = m.x.w - m.x.y;
        self.py_y = m.y.w - m.y.y;
        self.py_z = m.z.w - m.z.y;
        self.py_w = m.w.w - m.w.y;
        if (allow_test_spheres) {
            let invl = (self.py_x * self.py_x + self.py_y * self.py_y + self.py_z * self.py_z)
                .sqrt()
                .recip();
            self.py_x *= invl;
            self.py_y *= invl;
            self.py_z *= invl;
            self.py_w *= invl;
        }
        self.nz_x = m.x.w + m.x.z;
        self.nz_y = m.y.w + m.y.z;
        self.nz_z = m.z.w + m.z.z;
        self.nz_w = m.w.w + m.w.z;
        if (allow_test_spheres) {
            let invl = (self.nz_x * self.nz_x + self.nz_y * self.nz_y + self.nz_z * self.nz_z)
                .sqrt()
                .recip();
            self.nz_x *= invl;
            self.nz_y *= invl;
            self.nz_z *= invl;
            self.nz_w *= invl;
        }
        self.pz_x = m.x.w - m.x.z;
        self.pz_y = m.y.w - m.y.z;
        self.pz_z = m.z.w - m.z.z;
        self.pz_w = m.w.w - m.w.z;
        if (allow_test_spheres) {
            let invl = (self.pz_x * self.pz_x + self.pz_y * self.pz_y + self.pz_z * self.pz_z)
                .sqrt()
                .recip();
            self.pz_x *= invl;
            self.pz_y *= invl;
            self.pz_z *= invl;
            self.pz_w *= invl;
        }
    }

    /// Test wether a 3D point lies inside of the frustum
    pub fn test_point(&self, point: Vector3<S>) -> bool {
        self.nx_x * point.x + self.nx_y * point.y + self.nx_z * point.z + self.nx_w >= S::zero()
            && self.px_x * point.x + self.px_y * point.y + self.px_z * point.z + self.px_w
                >= S::zero()
            && self.ny_x * point.x + self.ny_y * point.y + self.ny_z * point.z + self.ny_w
                >= S::zero()
            && self.py_x * point.x + self.py_y * point.y + self.py_z * point.z + self.py_w
                >= S::zero()
            && self.nz_x * point.x + self.nz_y * point.y + self.nz_z * point.z + self.nz_w
                >= S::zero()
            && self.pz_x * point.x + self.pz_y * point.y + self.pz_z * point.z + self.pz_w
                >= S::zero()
    }

    /// Tests wether a sphere lies inside the frustum.
    ///
    /// This method doesn't distinguish between total and partial intersection with the frustum. In
    /// order to make this distinction, use the `intersect_sphere` method instead.
    pub fn test_sphere(&self, center: Vector3<S>, radius: S) -> bool {
        self.nx_x * center.x + self.nx_y * center.y + self.nx_z * center.z + self.nx_w >= -radius
            && self.px_x * center.x + self.px_y * center.y + self.px_z * center.z + self.px_w
                >= -radius
            && self.ny_x * center.x + self.ny_y * center.y + self.ny_z * center.z + self.ny_w
                >= -radius
            && self.py_x * center.x + self.py_y * center.y + self.py_z * center.z + self.py_w
                >= -radius
            && self.nz_x * center.x + self.nz_y * center.y + self.nz_z * center.z + self.nz_w
                >= -radius
            && self.pz_x * center.x + self.pz_y * center.y + self.pz_z * center.z + self.pz_w
                >= -radius
    }

    /// Test wether an axis aligned box, defined by is minimum (`min`) and maximum (`max`) points,
    /// lies inside or outside the frustum.
    ///
    /// This method won't distinguish between partial or total intersection. In order to obtain
    /// this information, use the `intersect_aab` method instead.
    pub fn test_aab(&self, min: Vector3<S>, max: Vector3<S>) -> bool {
        self.nx_x * if self.nx_x < S::zero() {
            min.x
        } else {
            max.x
        } + self.nx_y * if self.nx_y < S::zero() {
            min.y
        } else {
            max.y
        } + self.nx_z * if self.nx_z < S::zero() {
            min.z
        } else {
            max.z
        } >= -self.nx_w && self.px_x * if self.px_x < S::zero() {
            min.x
        } else {
            max.x
        } + self.px_y * if self.px_y < S::zero() {
            min.y
        } else {
            max.y
        } + self.px_z * if self.px_z < S::zero() {
            min.z
        } else {
            max.z
        } >= -self.px_w && self.ny_x * if self.ny_x < S::zero() {
            min.x
        } else {
            max.x
        } + self.ny_y * if self.ny_y < S::zero() {
            min.y
        } else {
            max.y
        } + self.ny_z * if self.ny_z < S::zero() {
            min.z
        } else {
            max.z
        } >= -self.ny_w && self.py_x * if self.py_x < S::zero() {
            min.x
        } else {
            max.x
        } + self.py_y * if self.py_y < S::zero() {
            min.y
        } else {
            max.y
        } + self.py_z * if self.py_z < S::zero() {
            min.z
        } else {
            max.z
        } >= -self.py_w && self.nz_x * if self.nz_x < S::zero() {
            min.x
        } else {
            max.x
        } + self.nz_y * if self.nz_y < S::zero() {
            min.y
        } else {
            max.y
        } + self.nz_z * if self.nz_z < S::zero() {
            min.z
        } else {
            max.z
        } >= -self.nz_w && self.pz_x * if self.pz_x < S::zero() {
            min.x
        } else {
            max.x
        } + self.pz_y * if self.pz_y < S::zero() {
            min.y
        } else {
            max.y
        } + self.pz_z * if self.pz_z < S::zero() {
            min.z
        } else {
            max.z
        } >= -self.pz_w
    }

    /// Returns the result of testing the intersection of the frustum with a sphere, defined by a
    /// center point (`center`) and a radius (`radius`).
    ///
    /// This method will distinguish between a partial intersection and a total intersection.
    pub fn intersect_sphere(&self, center: Vector3<S>, radius: S) -> IntersectionResult {
        let mut inside = true;
        let mut dist = S::zero();
        dist = self.nx_x * center.x + self.nx_y * center.y + self.nx_z * center.z + self.nx_w;
        if dist >= -radius {
            inside &= dist >= radius;
            dist = self.px_x * center.x + self.px_y * center.y + self.px_z * center.z + self.px_w;
            if dist >= -radius {
                inside &= dist >= radius;
                dist =
                    self.ny_x * center.x + self.ny_y * center.y + self.ny_z * center.z + self.ny_w;
                if dist >= -radius {
                    inside &= dist >= radius;
                    dist = self.py_x * center.x + self.py_y * center.y + self.py_z * center.z
                        + self.py_w;
                    if dist >= -radius {
                        inside &= dist >= radius;
                        dist = self.nz_x * center.x + self.nz_y * center.y + self.nz_z * center.z
                            + self.nz_w;
                        if dist >= -radius {
                            inside &= dist >= radius;
                            dist = self.pz_x * center.x + self.pz_y * center.y
                                + self.pz_z * center.z
                                + self.pz_w;
                            if dist >= -radius {
                                inside &= dist >= radius;
                                return if inside {
                                    IntersectionResult::Inside
                                } else {
                                    IntersectionResult::Intersect
                                };
                            }
                        }
                    }
                }
            }
        }

        IntersectionResult::Outside
    }

    pub fn intersect_aab(&self, min: Vector3<S>, max: Vector3<S>) -> IntersectionResult {
        let mut inside = true;
        if (self.nx_x * if self.nx_x < S::zero() {
            min.x
        } else {
            max.x
        } + self.nx_y * if self.nx_y < S::zero() {
            min.y
        } else {
            max.y
        } + self.nx_z * if self.nx_z < S::zero() {
            min.z
        } else {
            max.z
        } >= -self.nx_w)
        {
            inside &= self.nx_x * if self.nx_x < S::zero() {
                max.x
            } else {
                min.x
            } + self.nx_y * if self.nx_y < S::zero() {
                max.y
            } else {
                min.y
            } + self.nx_z * if self.nx_z < S::zero() {
                max.z
            } else {
                min.z
            } >= -self.nx_w;
            if (self.px_x * if self.px_x < S::zero() {
                min.x
            } else {
                max.x
            } + self.px_y * if self.px_y < S::zero() {
                min.y
            } else {
                max.y
            } + self.px_z * if self.px_z < S::zero() {
                min.z
            } else {
                max.z
            } >= -self.px_w)
            {
                inside &= self.px_x * if self.px_x < S::zero() {
                    max.x
                } else {
                    min.x
                } + self.px_y * if self.px_y < S::zero() {
                    max.y
                } else {
                    min.y
                } + self.px_z * if self.px_z < S::zero() {
                    max.z
                } else {
                    min.z
                } >= -self.px_w;
                if (self.ny_x * if self.ny_x < S::zero() {
                    min.x
                } else {
                    max.x
                } + self.ny_y * if self.ny_y < S::zero() {
                    min.y
                } else {
                    max.y
                } + self.ny_z * if self.ny_z < S::zero() {
                    min.z
                } else {
                    max.z
                } >= -self.ny_w)
                {
                    inside &= self.ny_x * if self.ny_x < S::zero() {
                        max.x
                    } else {
                        min.x
                    } + self.ny_y * if self.ny_y < S::zero() {
                        max.y
                    } else {
                        min.y
                    } + self.ny_z * if self.ny_z < S::zero() {
                        max.z
                    } else {
                        min.z
                    } >= -self.ny_w;
                    if (self.py_x * if self.py_x < S::zero() {
                        min.x
                    } else {
                        max.x
                    } + self.py_y * if self.py_y < S::zero() {
                        min.y
                    } else {
                        max.y
                    } + self.py_z * if self.py_z < S::zero() {
                        min.z
                    } else {
                        max.z
                    } >= -self.py_w)
                    {
                        inside &= self.py_x * if self.py_x < S::zero() {
                            max.x
                        } else {
                            min.x
                        } + self.py_y * if self.py_y < S::zero() {
                            max.y
                        } else {
                            min.y
                        } + self.py_z * if self.py_z < S::zero() {
                            max.z
                        } else {
                            min.z
                        } >= -self.py_w;
                        if (self.nz_x * if self.nz_x < S::zero() {
                            min.x
                        } else {
                            max.x
                        } + self.nz_y * if self.nz_y < S::zero() {
                            min.y
                        } else {
                            max.y
                        } + self.nz_z * if self.nz_z < S::zero() {
                            min.z
                        } else {
                            max.z
                        } >= -self.nz_w)
                        {
                            inside &= self.nz_x * if self.nz_x < S::zero() {
                                max.x
                            } else {
                                min.x
                            }
                                + self.nz_y * if self.nz_y < S::zero() {
                                    max.y
                                } else {
                                    min.y
                                }
                                + self.nz_z * if self.nz_z < S::zero() {
                                    max.z
                                } else {
                                    min.z
                                } >= -self.nz_w;
                            if (self.pz_x * if self.pz_x < S::zero() {
                                min.x
                            } else {
                                max.x
                            }
                                + self.pz_y * if self.pz_y < S::zero() {
                                    min.y
                                } else {
                                    max.y
                                }
                                + self.pz_z * if self.pz_z < S::zero() {
                                    min.z
                                } else {
                                    max.z
                                } >= -self.pz_w)
                            {
                                inside &= self.pz_x * if self.pz_x < S::zero() {
                                    max.x
                                } else {
                                    min.x
                                }
                                    + self.pz_y * if self.pz_y < S::zero() {
                                        max.y
                                    } else {
                                        min.y
                                    }
                                    + self.pz_z * if self.pz_z < S::zero() {
                                        max.z
                                    } else {
                                        min.z
                                    } >= -self.pz_w;
                                return if inside {
                                    IntersectionResult::Inside
                                } else {
                                    IntersectionResult::Intersect
                                };
                            }
                        }
                    }
                }
            }
        }

        IntersectionResult::Outside
    }
}

#[cfg(test)]
mod tests {
    use {FrustumIntersection, IntersectionResult};

    use cgmath::{Matrix4, Ortho, PerspectiveFov, Rad, Vector3, prelude::*};

    #[test]
    fn sphere_in_frustum_ortho() {
        let frustum_culling = FrustumIntersection::from_matrix4(
            Ortho {
                left: -1.0,
                right: 1.0,
                bottom: -1.0,
                top: 1.0,
                near: -1.0,
                far: 1.0,
            }.into(),
        );

        assert!(frustum_culling.test_sphere(Vector3::new(1.0, 0.0, 0.0), 0.1));
        assert!(!frustum_culling.test_sphere(Vector3::new(1.2, 0.0, 0.0), 0.1));
    }

    #[test]
    fn sphere_in_frustum_perspective() {
        let frustum_culling = FrustumIntersection::from_matrix4(
            PerspectiveFov {
                fovy: Rad(3.14159265 / 2.0),
                aspect: 1.0,
                near: 0.1,
                far: 100.0,
            }.into(),
        );

        assert!(frustum_culling.test_sphere(Vector3::new(1.0, 0.0, -2.0), 0.1));
        assert!(!frustum_culling.test_sphere(Vector3::new(4.0, 0.0, -2.0), 0.1));
    }

    #[test]
    fn test_point_in_perspective() {
        let frustum_culling = FrustumIntersection::from_matrix4(
            PerspectiveFov {
                fovy: Rad(3.14159265 / 2.0),
                aspect: 1.0,
                near: 0.1,
                far: 100.0,
            }.into(),
        );

        assert!(frustum_culling.test_point(Vector3::new(0.0, 0.0, -5.0)));
        assert!(!frustum_culling.test_point(Vector3::new(0.0, 6.0, -5.0)));
    }

    #[test]
    fn test_aab_in_ortho() {
        let mut c = FrustumIntersection::from_matrix4(
            Ortho {
                left: -1.0,
                right: 1.0,
                bottom: -1.0,
                top: 1.0,
                near: -1.0,
                far: 1.0,
            }.into(),
        );

        assert_eq!(
            IntersectionResult::Intersect,
            c.intersect_aab(Vector3::new(-20.0, -2.0, 0.0), Vector3::new(20.0, 2.0, 0.0))
        );
        assert_eq!(
            IntersectionResult::Inside,
            c.intersect_aab(Vector3::new(-0.5, -0.5, -0.5), Vector3::new(0.5, 0.5, 0.5))
        );
        assert_eq!(
            IntersectionResult::Outside,
            c.intersect_aab(Vector3::new(1.1, 0.0, 0.0), Vector3::new(2.0, 2.0, 2.0))
        );

        c.update(
            Ortho {
                left: -1.0,
                right: 1.0,
                bottom: -1.0,
                top: 1.0,
                near: -1.0,
                far: 1.0,
            }.into(),
        );

        assert_eq!(
            IntersectionResult::Intersect,
            c.intersect_aab(Vector3::new(0.0, 0.0, 0.0), Vector3::new(2.0, 2.0, 2.0))
        );
        assert_eq!(
            IntersectionResult::Outside,
            c.intersect_aab(Vector3::new(1.1, 0.0, 0.0), Vector3::new(2.0, 2.0, 2.0))
        );

        c.update(Matrix4::identity());

        assert_eq!(
            IntersectionResult::Intersect,
            c.intersect_aab(Vector3::new(0.5, 0.5, 0.5), Vector3::new(2.0, 2.0, 2.0))
        );
        assert_eq!(
            IntersectionResult::Outside,
            c.intersect_aab(Vector3::new(1.5, 0.5, 0.5), Vector3::new(2.0, 2.0, 2.0))
        );
        assert_eq!(
            IntersectionResult::Outside,
            c.intersect_aab(Vector3::new(-2.5, 0.5, 0.5), Vector3::new(-1.5, 2.0, 2.0))
        );
        assert_eq!(
            IntersectionResult::Outside,
            c.intersect_aab(Vector3::new(-0.5, -2.5, 0.5), Vector3::new(1.5, -2.0, 2.0))
        );
    }

    #[test]
    fn test_aab_in_perspective() {
        let c = FrustumIntersection::from_matrix4(
            PerspectiveFov {
                fovy: Rad(3.14159265 / 2.0),
                aspect: 1.0,
                near: 0.1,
                far: 100.0,
            }.into(),
        );

        assert_eq!(
            IntersectionResult::Inside,
            c.intersect_aab(Vector3::new(0.0, 0.0, -7.0), Vector3::new(1.0, 1.0, -5.0))
        );
        assert_eq!(
            IntersectionResult::Outside,
            c.intersect_aab(Vector3::new(1.1, 0.0, 0.0), Vector3::new(2.0, 2.0, 2.0))
        );
        assert_eq!(
            IntersectionResult::Outside,
            c.intersect_aab(Vector3::new(4.0, 4.0, -3.0), Vector3::new(5.0, 5.0, -5.0))
        );
        assert_eq!(
            IntersectionResult::Outside,
            c.intersect_aab(
                Vector3::new(-6.0, -6.0, -2.0),
                Vector3::new(-1.0, -4.0, -4.0)
            )
        );
    }
}
