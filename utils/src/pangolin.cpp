#include "utils/pangolin.hpp"

namespace pangolin {


void draw_camera(OpenGlMatrix &Twc, float w) {
  float h = w * 0.75, z = w * 0.6;
  glPushMatrix();
#ifdef HAVE_GLES
  glMultMatrixf(Twc.m);
#else
  glMultMatrixd(Twc.m);
#endif
  glBegin(GL_LINES);
  glVertex3f(0, 0, 0);
  glVertex3f(w, h, z);
  glVertex3f(0, 0, 0);
  glVertex3f(w, -h, z);
  glVertex3f(0, 0, 0);
  glVertex3f(-w, -h, z);
  glVertex3f(0, 0, 0);
  glVertex3f(-w, h, z);

  glVertex3f(w, h, z);
  glVertex3f(w, -h, z);
  glVertex3f(w, -h, z);
  glVertex3f(-w, -h, z);
  glVertex3f(-w, -h, z);
  glVertex3f(-w, h, z);
  glVertex3f(-w, h, z);
  glVertex3f(w, h, z);

  glEnd();
  glPopMatrix();
}

}
