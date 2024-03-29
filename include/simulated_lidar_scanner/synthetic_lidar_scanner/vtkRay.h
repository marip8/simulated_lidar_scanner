/*=========================================================================
 *
 *  Copyright David Doria 2011 daviddoria@gmail.com
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0.txt
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *=========================================================================*/

#ifndef __vtkRay_h
#define __vtkRay_h

#include <vtkObject.h>  //superclass

class vtkTransform;

/**
 * @brief This class is a container for a point and a direction - a very commonly used collection of objects.
 */
class vtkRay : public vtkObject
{
public:
  static vtkRay* New();
  vtkTypeMacro(vtkRay, vtkObject);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  vtkRay() = default;

  ////////// Accessors ///////////
  vtkGetVector3Macro(Origin, double);
  vtkGetVector3Macro(Direction, double);

  /////////// Mutators ///////////
  vtkSetVector3Macro(Origin, double);              // specify the origin of the ray
  void SetDirection(double* Dir);                  // specify the direction of the ray
  void ApplyTransform(vtkTransform* const Trans);  // transform the ray
  void GetPointAlong(const double dist,
                     double pointAlong[3]);  // get a point 'dist' units from the rays origin along the ray
  bool IsInfront(double* P);                 // check if a point is in the halfspace "in front of" the ray origin

private:
  double Origin[3]{ 0.0, 0.0, 0.0 };     //(x,y,z) coords of ray origin
  double Direction[3]{ 0.0, 0.0, 1.0 };  // the direction of the ray
};

#endif
