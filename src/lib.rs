extern crate cgmath;

use std::mem;

use cgmath::{
    BaseFloat,
    Matrix4,
    Vector3,
    Vector4,
    
    prelude::*,
};

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct FrustumIntersection<S> {
    nxX: S,
    nxY: S,
    nxZ: S,
    nxW: S,
    pxX: S,
    pxY: S,
    pxZ: S,
    pxW: S,
    nyX: S,
    nyY: S,
    nyZ: S,
    nyW: S,
    pyX: S,
    pyY: S,
    pyZ: S,
    pyW: S,
    nzX: S,
    nzY: S,
    nzZ: S,
    nzW: S,
    pzX: S,
    pzY: S,
    pzZ: S,
    pzW: S,
}

#[derive(Debug)]
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
        intersect.update(m, true);
        intersect
    }

    /// Updates the frustum, using a matrix, from which the frustum planes are extracted.
    ///
    /// If this FrustumIntersection will be used to test spheres, you can indicate so with
    /// the `allow_test_spheres` parameter
    pub fn update(&mut self, m: Matrix4<S>, allow_test_spheres: bool) {
        unimplemented!()
    }

    /// Test wether a 3D point lies inside of the frustum
    pub fn test_point(&self, point: Vector3<S>) -> bool {
        self.nxX * point.x + self.nxY * point.y + self.nxZ * point.z + self.nxW >= S::zero()
            && self.pxX * point.x + self.pxY * point.y + self.pxZ * point.z + self.pxW >= S::zero()
            && self.nyX * point.x + self.nyY * point.y + self.nyZ * point.z + self.nyW >= S::zero()
            && self.pyX * point.x + self.pyY * point.y + self.pyZ * point.z + self.pyW >= S::zero()
            && self.nzX * point.x + self.nzY * point.y + self.nzZ * point.z + self.nzW >= S::zero()
            && self.pzX * point.x + self.pzY * point.y + self.pzZ * point.z + self.pzW >= S::zero()
    }

    /// Tests wether a sphere lies inside the frustum.
    ///
    /// This method doesn't distinguish between total and partial intersection with the frustum. In
    /// order to make this distinction, use the `intersect_sphere` method instead.
    pub fn test_sphere(&self, center: Vector3<S>, radius: S) -> bool {
        self.nxX * center.x + self.nxY * center.y + self.nxZ * center.z + self.nxW >= -radius
            && self.pxX * center.x + self.pxY * center.y + self.pxZ * center.z + self.pxW >= -radius
            && self.nyX * center.x + self.nyY * center.y + self.nyZ * center.z + self.nyW >= -radius
            && self.pyX * center.x + self.pyY * center.y + self.pyZ * center.z + self.pyW >= -radius
            && self.nzX * center.x + self.nzY * center.y + self.nzZ * center.z + self.nzW >= -radius
            && self.pzX * center.x + self.pzY * center.y + self.pzZ * center.z + self.pzW >= -radius
    }

    /// Test wether an axis aligned box, defined by is minimum (`min`) and maximum (`max`) points,
    /// lies inside or outside the frustum.
    ///
    /// This method won't distinguish between partial or total intersection. In order to obtain
    /// this information, use the `intersect_aab` method instead.
    pub fn test_aab(&self, min: Vector3<S>, max: Vector3<S>) -> bool {
       self.nxX * if self.nxX < S::zero() { min.x } else { max.x } + self.nxY * if self.nxY < S::zero() { min.y } else { max.y }  + self.nxZ * if self.nxZ < S::zero() { min.z } else { max.z }  >= -self.nxW
           && self.pxX * if self.pxX < S::zero() { min.x } else { max.x } + self.pxY * if self.pxY < S::zero() { min.y } else { max.y }  + self.pxZ * if self.pxZ < S::zero() { min.z } else { max.z }  >= -self.pxW
           && self.nyX * if self.nyX < S::zero() { min.x } else { max.x } + self.nyY * if self.nyY < S::zero() { min.y } else { max.y }  + self.nyZ * if self.nyZ < S::zero() { min.z } else { max.z }  >= -self.nyW
           && self.pyX * if self.pyX < S::zero() { min.x } else { max.x } + self.pyY * if self.pyY < S::zero() { min.y } else { max.y }  + self.pyZ * if self.pyZ < S::zero() { min.z } else { max.z }  >= -self.pyW
           && self.nzX * if self.nzX < S::zero() { min.x } else { max.x } + self.nzY * if self.nzY < S::zero() { min.y } else { max.y }  + self.nzZ * if self.nzZ < S::zero() { min.z } else { max.z }  >= -self.nzW
           && self.pzX * if self.pzX < S::zero() { min.x } else { max.x } + self.pzY * if self.pzY < S::zero() { min.y } else { max.y }  + self.pzZ * if self.pzZ < S::zero() { min.z } else { max.z }  >= -self.pzW
    }

    /// Returns the result of testing the intersection of the frustum with a sphere, defined by a
    /// center point (`center`) and a radius (`radius`).
    ///
    /// This method will distinguish between a partial intersection and a total intersection.
    pub fn intersect_sphere(&self, center: Vector3<S>, radius: S) -> IntersectionResult {
        let mut inside = true;
        let mut dist = S::zero();
        dist = self.nxX * center.x + self.nxY * center.y + self.nxZ * center.z + self.nxW;
        if dist >= -radius {
            inside &= dist >= radius;
            dist = self.pxX * center.x + self.pxY * center.y + self.pxZ * center.z + self.pxW;
            if dist >= -radius {
                inside &= dist >= radius;
                dist = self.nyX * center.x + self.nyY * center.y + self.nyZ * center.z + self.nyW;
                if dist >= -radius {
                    inside &= dist >= radius;
                    dist = self.pyX * center.x + self.pyY * center.y + self.pyZ * center.z + self.pyW;
                    if dist >= -radius {
                        inside &= dist >= radius;
                        dist = self.nzX * center.x + self.nzY * center.y + self.nzZ * center.z + self.nzW;
                        if dist >= -radius {
                            inside &= dist >= radius;
                            dist = self.pzX * center.x + self.pzY * center.y + self.pzZ * center.z + self.pzW;
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

    pub fn intersect_aab(&self, min: &Vector3<S>, max: &Vector3<S>) -> IntersectionResult {
        let mut inside = true;
        if (self.nxX * if self.nxX < S::zero() { min.x } else { max.x } + self.nxY * if self.nxY < S::zero() { min.y } else { max.y } + self.nxZ * if self.nxZ < S::zero() { min.z } else { max.z } >= -self.nxW) {
            inside &= self.nxX * if self.nxX < S::zero() { max.x } else { min.x } + self.nxY * if self.nxY < S::zero() { max.y } else { min.y } + self.nxZ * if self.nxZ < S::zero() { max.z } else { min.z } >= -self.nxW;
            if (self.pxX * if self.pxX < S::zero() { min.x } else { max.x } + self.pxY * if self.pxY < S::zero() { min.y } else { max.y } + self.pxZ * if self.pxZ < S::zero() { min.z } else { max.z } >= -self.pxW) {
                inside &= self.pxX * if self.pxX < S::zero() { max.x } else { min.x } + self.pxY * if self.pxY < S::zero() { max.y } else { min.y } + self.pxZ * if self.pxZ < S::zero() { max.z } else { min.z } >= -self.pxW;
                if (self.nyX * if self.nyX < S::zero() { min.x } else { max.x } + self.nyY * if self.nyY < S::zero() { min.y } else { max.y } + self.nyZ * if self.nyZ < S::zero() { min.z } else { max.z } >= -self.nyW) {
                    inside &= self.nyX * if self.nyX < S::zero() { max.x } else { min.x } + self.nyY * if self.nyY < S::zero() { max.y } else { min.y } + self.nyZ * if self.nyZ < S::zero() { max.z } else { min.z } >= -self.nyW;
                    if (self.pyX * if self.pyX < S::zero() { min.x } else { max.x } + self.pyY * if self.pyY < S::zero() { min.y } else { max.y } + self.pyZ * if self.pyZ < S::zero() { min.z } else { max.z } >= -self.pyW) {
                        inside &= self.pyX * if self.pyX < S::zero() { max.x } else { min.x } + self.pyY * if self.pyY < S::zero() { max.y } else { min.y } + self.pyZ * if self.pyZ < S::zero() { max.z } else { min.z } >= -self.pyW;
                        if (self.nzX * if self.nzX < S::zero() { min.x } else { max.x } + self.nzY * if self.nzY < S::zero() { min.y } else { max.y } + self.nzZ * if self.nzZ < S::zero() { min.z } else { max.z } >= -self.nzW) {
                            inside &= self.nzX * if self.nzX < S::zero() { max.x } else { min.x } + self.nzY * if self.nzY < S::zero() { max.y } else { min.y } + self.nzZ * if self.nzZ < S::zero() { max.z } else { min.z } >= -self.nzW;
                            if (self.pzX * if self.pzX < S::zero() { min.x } else { max.x } + self.pzY * if self.pzY < S::zero() { min.y } else { max.y } + self.pzZ * if self.pzZ < S::zero() { min.z } else { max.z } >= -self.pzW) {
                                inside &= self.pzX * if self.pzX < S::zero() { max.x } else { min.x } + self.pzY * if self.pzY < S::zero() { max.y } else { min.y } + self.pzZ * if self.pzZ < S::zero() { max.z } else { min.z } >= -self.pzW;
                                return if inside {
                                    IntersectionResult::Inside
                                } else {
                                    IntersectionResult::Inside
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
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
