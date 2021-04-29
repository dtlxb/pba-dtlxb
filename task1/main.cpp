
#include <GLFW/glfw3.h>
#include "delfem2/glfw/viewer2.h"
#include "delfem2/color.h"
#include <random>
#include <cstdlib>
#include <cstdio>
float M_PI = 3.1415926;

// print out error
static void error_callback(int error, const char* description){
  fputs(description, stderr);
}

class CParticle{
public:
  float pos[2] = {0.f, 0.f};
  float velo[2] = {0.f, 0.f};
  float color[3] = {0.f, 0.f, 0.f};
};

void draw(
    const std::vector<CParticle>& aParticle)
{
  ::glLineWidth(2);

  // draw boundary
  ::glBegin(GL_LINE_LOOP);
  ::glColor3f(0.f, 0.f, 0.f);
  ::glVertex2f(0.f, 0.f);
  ::glVertex2f(1.f, 0.f);
  ::glVertex2f(1.f, 1.f);
  ::glVertex2f(0.f, 1.f);
  ::glEnd();

  { // draw obstacle circle (center=(0.5,0.5) radius=0.2 )
    ::glBegin(GL_LINE_LOOP);
    ::glColor3f(0.f, 0.f, 0.f);
    const int ndiv = 64;
    const float dr = 2.f*M_PI/ndiv;
    for(unsigned int idiv=0;idiv<ndiv;++idiv){
      ::glVertex2f(
          0.5+0.2*cos(dr*float(idiv)),
          0.5+0.2*sin(dr*float(idiv)) );
    }
    ::glEnd();
  }

  // draw points
  ::glPointSize(10);
  ::glBegin(GL_POINTS);
  for(const auto & ip : aParticle){
    ::glColor3fv(ip.color);
    ::glVertex2d(ip.pos[0],ip.pos[1]);
  }
  ::glEnd();
}

void simulate(std::vector<CParticle>& aParticle,float dt)
{ for(auto & p : aParticle){
    p.pos[0] += p.velo[0]*dt;
    p.pos[1] += p.velo[1]*dt;
  }
  for(auto & p : aParticle){
    // solve collisions below
    if( p.pos[0] < 0 ){ // left wall
      p.pos[0] = -p.pos[0];
      p.velo[0] = -p.velo[0];
    }
    if( p.pos[1] < 0 ){ // bottom wall
      p.velo[1] = -p.velo[1];
    }
    if( p.pos[0] > 1 ){ // left wall
      p.pos[0] = 2-p.pos[0];
      p.velo[0] = -p.velo[0];
    }
    if( p.pos[1] > 1 ){ // top wall
      p.pos[1] = 2-p.pos[1];
      p.velo[1] = -p.velo[1];
    }

    float dx = p.pos[0]-0.5f;
    float dy = p.pos[1]-0.5f; // y-coord from center
    float dist_from_center = sqrt(dx*dx+dy*dy);

    if( dist_from_center < 0.2 ){ // collision with obstacle
      std::cout<<"Collision!"<<std::endl;
      std::cout<<"Before Pos:("<<p.pos[0]<<", "<<p.pos[1]<<")"<<"Before Velo:("<<p.velo[0]<<", "<<p.velo[1]<<")"<<std::endl;
      float norm[2] = {dx/dist_from_center, dy/dist_from_center }; // unit normal vector of the circle
      std::cout<<"Norm:("<<norm[0]<<", "<<norm[1]<<")"<<std::endl;
      float vnorm = p.velo[0]*norm[0] + p.velo[1]*norm[1]; // normal component of the velocity
        ////////////////////////////
        // write something below !

        // roll back. This is not the TRUE positionW. 
        //while (sqrt((p.pos[0]-0.5f)*(p.pos[0]-0.5f)+(p.pos[1]-0.5f)*(p.pos[1]-0.5f))<0.2){
        p.pos[0] -= p.velo[0] * dt;p.pos[1] -= p.velo[1] * dt;
        //}
        //p.pos[0] = 0.5 + 2*norm[0]*norm[1]* (p.pos[1]-0.5) + (norm[0]*norm[0] - norm[1]*norm[1]) * (p.pos[0]-0.5);
        //p.pos[1] = 0.5 + 2*norm[0]*norm[1]* (p.pos[0]-0.5) + ( - norm[0]*norm[0] + norm[1]*norm[1]) * (p.pos[1]-0.5);
        float v_abs = sqrt(p.velo[0] * p.velo[0] + p.velo[1] * p.velo[1]);
        p.velo[0] = 2*norm[0]*norm[1]* (-p.velo[1]) + (norm[0]*norm[0] - norm[1]*norm[1]) * (-p.velo[0]);
        p.velo[1] = 2*norm[0]*norm[1]* (-p.velo[0]) + ( - norm[0]*norm[0] + norm[1]*norm[1]) * (-p.velo[1]);
        p.velo[0] /= norm[0]*norm[0] + norm[1]*norm[1];
        p.velo[1] /= norm[0]*norm[0] + norm[1]*norm[1];
        float v_new_abs = sqrt(p.velo[0] * p.velo[0] + p.velo[1] * p.velo[1]);
        float ratio = v_abs/v_new_abs;
        p.velo[0] *= ratio; p.velo[1] *= ratio;

      std::cout<<"After Pos:("<<p.pos[0]<<", "<<p.pos[1]<<")"<<"After Velo:("<<p.velo[0]<<", "<<p.velo[1]<<")"<<std::endl<<std::endl;
      }

    }

  
}



int main()
{
  const unsigned int N = 50; // number of points
  const float dt = 0.01;
  std::vector<CParticle> aParticle;
  { // initialize particle
    std::mt19937 rndeng(std::random_device{}());
    std::uniform_real_distribution<float> dist01(0,1);
    aParticle.resize(N);
    for(auto & p : aParticle){
      // set position
      p.pos[0] = dist01(rndeng);
      p.pos[1] = dist01(rndeng);
      // set color
      delfem2::GetRGB_HSV(
          p.color[0], p.color[1], p.color[2],
          dist01(rndeng), 1.f, 1.f);
      // set velocity
      p.velo[0] = dist01(rndeng)*2.f-1.f;
      p.velo[1] = dist01(rndeng)*2.f-1.f;
      const float lenvelo = sqrt(p.velo[0]*p.velo[0]+p.velo[1]*p.velo[1]);
      p.velo[0] /= lenvelo;
      p.velo[1] /= lenvelo;
    }
  }
  delfem2::glfw::CViewer2 viewer;
  { // specify view
    viewer.view_height = 1.0;
    viewer.trans[0] = -0.5;
    viewer.trans[1] = -0.5;
    viewer.title = "task1";
  }
  glfwSetErrorCallback(error_callback);
  // -----------------------------------
  // below: opengl starts from here
  if ( !glfwInit() ) { exit(EXIT_FAILURE); }
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
  viewer.InitGL();

  while (!glfwWindowShouldClose(viewer.window))
  {
    viewer.DrawBegin_oldGL();
    simulate(aParticle,dt);
    draw(aParticle);
    viewer.SwapBuffers();
    glfwPollEvents();
  }
  glfwDestroyWindow(viewer.window);
  glfwTerminate();
  exit(EXIT_SUCCESS);
}
