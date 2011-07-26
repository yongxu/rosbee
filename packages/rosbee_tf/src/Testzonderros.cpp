// Just some math to turn wheel odometry into position updates
// Released into the public domain 3 June 2010

#include <stdio.h>
#include <math.h>
#include <stdlib.h>


#define WHEELBASE 0.41
// left wheel
double Lx = -WHEELBASE/2.0;
double Ly = 0.0;

// right wheel
double Rx = WHEELBASE/2.0;
double Ry = 0.0;


// given distances traveled by each wheel, updates the
// wheel position globals
void update_wheel_position(double l, double r) {

  if (fabs(r - l) < 0.001) {
    // If both wheels moved about the same distance, then we get an infinite
    // radius of curvature.  This handles that case.

    // find forward by rotating the axle between the wheels 90 degrees
    double axlex = Rx - Lx;
    double axley = Ry - Ly;

    double forwardx, forwardy;
    forwardx = -axley;
    forwardy = axlex;

    // normalize
    double length = sqrt(forwardx*forwardx + forwardy*forwardy);
    forwardx = forwardx / length;
    forwardy = forwardy / length;

    // move each wheel forward by the amount it moved
    Lx = Lx + forwardx * l;
    Ly = Ly + forwardy * l;

    Rx = Rx + forwardx * r;
    Ry = Ry + forwardy * r;
    
    return;
  }

  double rl; // radius of curvature for left wheel
  rl = WHEELBASE * l / (r - l);

  printf("Radius of curvature (left wheel): %.2lf\n", rl);

  double theta; // angle we moved around the circle, in radians
  // theta = 2 * PI * (l / (2 * PI * rl)) simplifies to:
  theta = l / rl;

  printf("Theta: %.2lf radians\n", theta);

  // Find the point P that we're circling
  double Px, Py;

  Px = Lx + rl*((Lx-Rx)/WHEELBASE);
  Py = Ly + rl*((Ly-Ry)/WHEELBASE);

  printf("Center of rotation: (%.2lf, %.2lf)\n", Px, Py);

  // Translate everything to the origin
  double Lx_translated = Lx - Px;
  double Ly_translated = Ly - Py;

  double Rx_translated = Rx - Px;
  double Ry_translated = Ry - Py;

  printf("Translated: (%.2lf,%.2lf) (%.2lf,%.2lf)\n",
    Lx_translated, Ly_translated,
    Rx_translated, Ry_translated);

  // Rotate by theta
  double cos_theta = cos(theta);
  double sin_theta = sin(theta);

  printf("cos(theta)=%.2lf sin(theta)=%.2lf\n", cos_theta, sin_theta);

  double Lx_rotated = Lx_translated*cos_theta - Ly_translated*sin_theta;
  double Ly_rotated = Lx_translated*sin_theta + Ly_translated*sin_theta;

  double Rx_rotated = Rx_translated*cos_theta - Ry_translated*sin_theta;
  double Ry_rotated = Rx_translated*sin_theta + Ry_translated*sin_theta;

  printf("Rotated: (%.2lf,%.2lf) (%.2lf,%.2lf)\n",
    Lx_rotated, Ly_rotated,
    Rx_rotated, Ry_rotated);

  // Translate back
  Lx = Lx_rotated + Px;
  Ly = Ly_rotated + Py;

  Rx = Rx_rotated + Px;
  Ry = Ry_rotated + Py;
}

main(int argc, char **argv) {
  if (argc != 3) {
    printf("Usage: %s left right\nwhere left and right are distances.\n",
      argv[0]);
    return 1;
  }

  double left = atof(argv[1]);
  double right = atof(argv[2]);

  printf("Old wheel positions: (%lf,%lf) (%lf,%lf)\n",
    Lx, Ly, Rx, Ry);
  update_wheel_position(left, right);
  printf("New wheel positions: (%lf,%lf) (%lf,%lf)\n",
    Lx, Ly, Rx, Ry);
}


