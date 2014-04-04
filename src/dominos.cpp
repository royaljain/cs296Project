/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

/* 
 * Base code for CS 296 Software Systems Lab 
 * Department of Computer Science and Engineering, IIT Bombay
 * Instructor: Parag Chaudhuri
 */



#include "cs296_base.hpp"
#include "render.hpp"
#define DEGTORAD 0.0174532925199432957f
#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include "GL/freeglut.h"
#endif
 #include<math.h>
#include<iostream>
#include <cstring>
using namespace std;

#include "dominos.hpp"
#define PI 3.14159


int initialAngle = 0;
int finalAngle = 0;

b2Body* body ;
b2Body* output;
b2Body* governer; 




namespace cs296
{
  /**  The is the constructor 
   * This is the documentation block for the constructor.
   */ 
  

b2Body* generateSpokedWheel(float radius ,float x_centre,float y_centre,float angle,b2World* m_world,b2Body* b2, int spokes)
{
      b2Body *b;
  
      b2BodyDef bd;
      bd.position.Set(x_centre,y_centre);
      bd.type = b2_dynamicBody;
      b = m_world->CreateBody(&bd);
      
      b2CircleShape circle;
      circle.m_radius = radius;

      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.0f;
      fd->shape = &circle;
      b->CreateFixture(fd);

/*
      b2BodyDef bd2;
      bd2.type = b2_staticBody;
      bd2.position.Set(x_centre,y_centre);
      b2 = m_world->CreateBody(&bd2);
      
      b2CircleShape circle2;
      circle2.m_radius = 1.1f;

      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 1.0f;
      fd2->shape = &circle2;
      b2->CreateFixture(fd2);
*/
      




      for(int i=0;i<spokes;i++)
      {

        float ang = 360.0/spokes;        
        float theta = (ang*i + angle)*PI/180 ;
        float deltheta = ang*PI/360;
        float length = deltheta*radius/2.0;

        b2PolygonShape bs1;
        b2Vec2 pos((radius*cos(deltheta/2.0)+length/2.0)*cos(theta + deltheta/2.0),(radius*cos(deltheta/2.0)+length/2.0)*sin(theta + deltheta/2.0));
        bs1.SetAsBox(2.0*length,length,pos,theta+deltheta/2);


        b2FixtureDef *fd2 = new b2FixtureDef;
        fd2->density = 1.0f;
        fd2->shape = &bs1;
      
        b->CreateFixture(fd2); 
      }



       b2RevoluteJointDef revoluteJointDef;
       revoluteJointDef.Initialize(b,b2,b->GetWorldCenter());
       m_world->CreateJoint(&revoluteJointDef);

        
        return b;
}





  dominos_t::dominos_t()
  {

    b2Body* reference;  
      b2BodyDef bd;
      bd.type = b2_staticBody;
      bd.position.Set(0.0f,0.0f);
      reference = m_world->CreateBody(&bd);
      
      b2CircleShape circle2;
      circle2.m_radius = 1.1f;

      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 100000.0f;
      fd2->shape = &circle2;
      reference->CreateFixture(fd2);

      {
        b2Body *stopper;
    
        b2BodyDef bdstop;
        bdstop.position.Set(28.5f,0);
        bdstop.type = b2_dynamicBody;
        stopper = m_world->CreateBody(&bdstop);
        
        b2PolygonShape stpshape;
        b2Vec2 posstop(15.0f,23.5f);
        stpshape.SetAsBox(5.0f,0.25f,posstop,0);

        b2FixtureDef *fdstop = new b2FixtureDef;
        fdstop->density = 100.0f;
        fdstop->shape = &stpshape;
        stopper->CreateFixture(fdstop);




        b2Body* shelf;
        b2BodyDef bd;
        bd.type = b2_staticBody;
        bd.position.Set(28.5f,0.0f);
        shelf = m_world->CreateBody(&bd);
        
        b2PolygonShape rect;
        b2Vec2 pos(18.0f,22.0f);
        rect.SetAsBox(3.0f,1.0f,pos,0);

        b2FixtureDef *fd2 = new b2FixtureDef;
        fd2->density = 100000.0f;
        fd2->shape = &rect;
        shelf->CreateFixture(fd2);



        b2RevoluteJointDef revoluteJointDef;
        revoluteJointDef.localAnchorA.Set(5.0f,0.0f);
       revoluteJointDef.Initialize(reference,stopper,stopper->GetWorldCenter());
       m_world->CreateJoint(&revoluteJointDef);

      }


 
    { 
      float radius_input = 4.0f;
      float centre_x_input = 35.0f;
      float centre_y_input = 10.0f;
      
      body = generateSpokedWheel(radius_input,centre_x_input,centre_y_input,0.0,m_world,reference,10);
      
      float radius_governer = 8.0f;
      float centre_x_governer = 30.0f;
      float distance = (radius_governer + radius_input + max(radius_governer,radius_input)*PI/20 + 0.1*min(radius_governer,radius_input));
      float centre_y_governer = sqrt(distance*distance - pow((centre_x_input - centre_x_governer),2)) + centre_y_input; 
      
      float radius_output = 4.0f;
      float centre_x_output = 30.0f;
      float distance2= (radius_governer + radius_output + max(radius_governer,radius_output)*PI/20 + 0.1*min(radius_governer,radius_output));
      float centre_y_output = centre_y_governer + sqrt(distance2*distance2 - pow((centre_x_output - centre_x_governer),2));

      governer = generateSpokedWheel(radius_governer,centre_x_governer,centre_y_governer,18.0,m_world,reference,20);
      output = generateSpokedWheel(radius_output,centre_x_output,centre_y_output,0.0,m_world,reference,10);

    }  

      


    }


  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
