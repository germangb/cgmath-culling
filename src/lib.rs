extern crate cgmath;

use std::mem;

use cgmath::{
    BaseFloat,

    Perspective,
    PerspectiveFov,
    Ortho,

    Matrix4,
    Vector3,
    Vector4,
    
    prelude::*,
};

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct FrustumCuller<S> {
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
pub enum Intersection {
    /// fully inside the frustum
    Inside,
    /// Partially inside the frustum
    Partial,
    /// Fully outside the frustum
    Outside,
}

impl<S: BaseFloat> FrustumCuller<S> {
    #[inline]
    pub fn from_perspective(perspective: Perspective<S>) -> Self {
        Self::from_matrix(perspective.into())
    }

    #[inline]
    pub fn from_perspective_fov(perspective: PerspectiveFov<S>) -> Self {
        Self::from_matrix(perspective.into())
    }

    #[inline]
    pub fn from_ortho(ortho: Ortho<S>) -> Self {
        Self::from_matrix(ortho.into())
    }

    pub fn from_matrix(m: Matrix4<S>) -> Self {
        let mut culler: Self = unsafe { mem::zeroed() };

        culler.nx_x = m.x.w + m.x.x;
        culler.nx_y = m.y.w + m.y.x;
        culler.nx_z = m.z.w + m.z.x;
        culler.nx_w = m.w.w + m.w.x;
        //if (allow_test_spheres) {
            let invl = (culler.nx_x * culler.nx_x + culler.nx_y * culler.nx_y + culler.nx_z * culler.nx_z)
                .sqrt()
                .recip();
            culler.nx_x *= invl;
            culler.nx_y *= invl;
            culler.nx_z *= invl;
            culler.nx_w *= invl;
        //}
        culler.px_x = m.x.w - m.x.x;
        culler.px_y = m.y.w - m.y.x;
        culler.px_z = m.z.w - m.z.x;
        culler.px_w = m.w.w - m.w.x;
        //if (allow_test_spheres) {
            let invl = (culler.px_x * culler.px_x + culler.px_y * culler.px_y + culler.px_z * culler.px_z)
                .sqrt()
                .recip();
            culler.px_x *= invl;
            culler.px_y *= invl;
            culler.px_z *= invl;
            culler.px_w *= invl;
        //}
        culler.ny_x = m.x.w + m.x.y;
        culler.ny_y = m.y.w + m.y.y;
        culler.ny_z = m.z.w + m.z.y;
        culler.ny_w = m.w.w + m.w.y;
        //if (allow_test_spheres) {
            let invl = (culler.ny_x * culler.ny_x + culler.ny_y * culler.ny_y + culler.ny_z * culler.ny_z)
                .sqrt()
                .recip();
            culler.ny_x *= invl;
            culler.ny_y *= invl;
            culler.ny_z *= invl;
            culler.ny_w *= invl;
        //}
        culler.py_x = m.x.w - m.x.y;
        culler.py_y = m.y.w - m.y.y;
        culler.py_z = m.z.w - m.z.y;
        culler.py_w = m.w.w - m.w.y;
        //if (allow_test_spheres) {
            let invl = (culler.py_x * culler.py_x + culler.py_y * culler.py_y + culler.py_z * culler.py_z)
                .sqrt()
                .recip();
            culler.py_x *= invl;
            culler.py_y *= invl;
            culler.py_z *= invl;
            culler.py_w *= invl;
        //}
        culler.nz_x = m.x.w + m.x.z;
        culler.nz_y = m.y.w + m.y.z;
        culler.nz_z = m.z.w + m.z.z;
        culler.nz_w = m.w.w + m.w.z;
        //if (allow_test_spheres) {
            let invl = (culler.nz_x * culler.nz_x + culler.nz_y * culler.nz_y + culler.nz_z * culler.nz_z)
                .sqrt()
                .recip();
            culler.nz_x *= invl;
            culler.nz_y *= invl;
            culler.nz_z *= invl;
            culler.nz_w *= invl;
        //}
        culler.pz_x = m.x.w - m.x.z;
        culler.pz_y = m.y.w - m.y.z;
        culler.pz_z = m.z.w - m.z.z;
        culler.pz_w = m.w.w - m.w.z;
        //if (allow_test_spheres) {
            let invl = (culler.pz_x * culler.pz_x + culler.pz_y * culler.pz_y + culler.pz_z * culler.pz_z)
                .sqrt()
                .recip();
            culler.pz_x *= invl;
            culler.pz_y *= invl;
            culler.pz_z *= invl;
            culler.pz_w *= invl;
        //}
        
        culler
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
    pub fn intersect_sphere(&self, center: Vector3<S>, radius: S) -> Intersection {
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
                                    Intersection::Inside
                                } else {
                                    Intersection::Partial
                                };
                            }
                        }
                    }
                }
            }
        }

        Intersection::Outside
    }

    pub fn intersect_aab(&self, min: Vector3<S>, max: Vector3<S>) -> Intersection {
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
                                    Intersection::Inside
                                } else {
                                    Intersection::Partial
                                };
                            }
                        }
                    }
                }
            }
        }

        Intersection::Outside
    }
}

#[cfg(test)]
mod tests {
    use {FrustumCuller, Intersection};

    use cgmath::{Matrix4, Ortho, PerspectiveFov, Rad, Vector3, prelude::*};

    #[test]
    fn sphere_in_frustum_ortho() {
        let frustum_culling = FrustumCuller::from_matrix(
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
        let frustum_culling = FrustumCuller::from_matrix(
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
        let frustum_culling = FrustumCuller::from_matrix(
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
        let mut c = FrustumCuller::from_matrix(
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
            Intersection::Partial,
            c.intersect_aab(Vector3::new(-20.0, -2.0, 0.0), Vector3::new(20.0, 2.0, 0.0))
        );
        assert_eq!(
            Intersection::Inside,
            c.intersect_aab(Vector3::new(-0.5, -0.5, -0.5), Vector3::new(0.5, 0.5, 0.5))
        );
        assert_eq!(
            Intersection::Outside,
            c.intersect_aab(Vector3::new(1.1, 0.0, 0.0), Vector3::new(2.0, 2.0, 2.0))
        );

        c = FrustumCuller::from_ortho(
            Ortho {
                left: -1.0,
                right: 1.0,
                bottom: -1.0,
                top: 1.0,
                near: -1.0,
                far: 1.0,
            },
        );

        assert_eq!(
            Intersection::Partial,
            c.intersect_aab(Vector3::new(0.0, 0.0, 0.0), Vector3::new(2.0, 2.0, 2.0))
        );
        assert_eq!(
            Intersection::Outside,
            c.intersect_aab(Vector3::new(1.1, 0.0, 0.0), Vector3::new(2.0, 2.0, 2.0))
        );

        c = FrustumCuller::from_matrix(Matrix4::identity());

        assert_eq!(
            Intersection::Partial,
            c.intersect_aab(Vector3::new(0.5, 0.5, 0.5), Vector3::new(2.0, 2.0, 2.0))
        );
        assert_eq!(
            Intersection::Outside,
            c.intersect_aab(Vector3::new(1.5, 0.5, 0.5), Vector3::new(2.0, 2.0, 2.0))
        );
        assert_eq!(
            Intersection::Outside,
            c.intersect_aab(Vector3::new(-2.5, 0.5, 0.5), Vector3::new(-1.5, 2.0, 2.0))
        );
        assert_eq!(
            Intersection::Outside,
            c.intersect_aab(Vector3::new(-0.5, -2.5, 0.5), Vector3::new(1.5, -2.0, 2.0))
        );
    }

    #[test]
    fn test_aab_in_perspective() {
        let c = FrustumCuller::from_perspective_fov(
            PerspectiveFov {
                fovy: Rad(3.14159265 / 2.0),
                aspect: 1.0,
                near: 0.1,
                far: 100.0,
            },
        );

        assert_eq!(
            Intersection::Inside,
            c.intersect_aab(Vector3::new(0.0, 0.0, -7.0), Vector3::new(1.0, 1.0, -5.0))
        );
        assert_eq!(
            Intersection::Outside,
            c.intersect_aab(Vector3::new(1.1, 0.0, 0.0), Vector3::new(2.0, 2.0, 2.0))
        );
        assert_eq!(
            Intersection::Outside,
            c.intersect_aab(Vector3::new(4.0, 4.0, -3.0), Vector3::new(5.0, 5.0, -5.0))
        );
        assert_eq!(
            Intersection::Outside,
            c.intersect_aab(
                Vector3::new(-6.0, -6.0, -2.0),
                Vector3::new(-1.0, -4.0, -4.0)
            )
        );
    }
}
