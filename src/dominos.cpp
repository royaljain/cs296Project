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
#include<stdio.h>
#include <cstring>
using namespace std;

#include "dominos.hpp"
#define PI 3.14159


int initialAngle = 0;
int finalAngle = 0;

int initialAngle2 = 0;
int finalAngle2 = 0;

b2Body* reference;
b2World* m_world;
float y_ca;
b2Body* body ;
b2Body* dirchanger;
b2Body* output;
b2Body* governer; 
b2Body* carry_support; 
b2RevoluteJoint* m_joint;
b2RevoluteJoint* m_joint2;
b2Body* b;
b2Body* body2 ;
b2Body* output2;
b2Body* governer2; 
b2Body* shelf;
b2Body* shelf2;


namespace cs296
{
  /**  The is the constructor 
   * This is the documentation block for the constructor.
   */ 
  

b2Body* generateSpokedWheel(int grpIndex, float radius ,float x_centre,float y_centre,float angle,b2World* m_world,b2Body* b2, int spokes, float dens)
{
      b2Body *b;
  
      b2BodyDef bd;
      bd.position.Set(x_centre,y_centre);
      bd.type = b2_dynamicBody;
      b = m_world->CreateBody(&bd);
      
      b2CircleShape circle;
      circle.m_radius = radius;

      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = dens;
      fd->shape = &circle;
      fd->filter.groupIndex= grpIndex;
      b->CreateFixture(fd);

      for(int i=0;i<spokes;i++)
      {
        float ang = 360.0/spokes;        
        float theta = (ang*i + angle)*PI/180 ;
        float deltheta = ang*PI/720;
        float length = 2*radius*sin(deltheta/2.0);

        b2PolygonShape bs1;
        b2Vec2 pos((radius*cos(deltheta/2.0)+length)*cos(theta + deltheta/2.0),
          (radius*cos(deltheta/2.0)+length)*sin(theta + deltheta/2.0));
        bs1.SetAsBox(length,length,pos,theta+deltheta/2);


        b2FixtureDef *fd2 = new b2FixtureDef;
        fd2->density = 1.0f;
        fd2->shape = &bs1;
        fd2->filter.groupIndex= grpIndex;
        b->CreateFixture(fd2); 
  
        b2PolygonShape bs2;
        
        
        b2Vec2 vertices[3];

        vertices[0].Set(pos.x + length*cos(theta + deltheta/2.0)+length*sin(theta + deltheta/2.0),
        pos.y + length*sin(theta + deltheta/2.0)-length*cos(theta + deltheta/2.0));

        vertices[1].Set(pos.x + 2*length*cos(theta + deltheta/2.0),pos.y + 2*length*sin(theta + deltheta/2.0));
        
        vertices[2].Set(pos.x +length*cos(theta + deltheta/2.0)-length*sin(theta + deltheta/2.0),
        pos.y + length*sin(theta + deltheta/2.0)+length*cos(theta + deltheta/2.0));

        bs2.Set(vertices,3);


        b2FixtureDef *fd3 = new b2FixtureDef;
        fd3->density = 1.0f;
        fd3->shape = &bs2;
        fd3->filter.groupIndex= grpIndex;
        b->CreateFixture(fd3);




      }

       b2RevoluteJointDef revoluteJointDef;
       revoluteJointDef.Initialize(b,b2,b->GetWorldCenter());
       m_world->CreateJoint(&revoluteJointDef);
       return b;
}


b2Body* generateTwoSpokedWheel(b2Body* gov,b2World* m_world,int grpIndex)
{


      float radius_input = 4.0f;
      float centre_x_input = 35.0f;
      float centre_y_input = 10.0f;
      
      
      float radius_governer = 8.0f;
      float centre_x_governer = 30.0f;
      float distance = (radius_governer + radius_input + max(radius_governer,radius_input)*PI/20 + 0.1*min(radius_governer,radius_input));
      float centre_y_governer = sqrt(distance*distance - pow((centre_x_input - centre_x_governer),2)) + centre_y_input; 
      

      b2Body* b;

      float radius = 4.000f;
      float centre_x_carry_support = 8.4f;
       
      b2BodyDef bd;
      bd.position.Set(centre_x_carry_support,centre_y_governer);
      bd.type = b2_dynamicBody;
      
      
      b = m_world->CreateBody(&bd);
    

      b2CircleShape circle;
      circle.m_radius = radius;

      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 30.0f;
      fd->shape = &circle;
      fd->filter.groupIndex=grpIndex;
      b->CreateFixture(fd);


      float spokes = 5.0;
      float ang = 360.0/spokes;        
      float angl=20;
      float theta = (144+angl)*PI/180;
      float deltheta = ang*PI/720;
      float length = radius*sin(deltheta/2.0);
      int si = 4.8;

      for(int i=0;i<2;i++){

        b2PolygonShape bs1;
        b2Vec2 pos((radius*cos(deltheta/2.0)+length)*cos(theta + deltheta/2.0),
        (radius*cos(deltheta/2.0)+length)*sin(theta + deltheta/2.0));
        bs1.SetAsBox((si)*length,length,pos,theta+deltheta/2);


        b2FixtureDef *fd2 = new b2FixtureDef;
        fd2->filter.groupIndex=grpIndex;
        fd2->density = 40.0f;
        fd2->shape = &bs1;
  
        b->CreateFixture(fd2);

        b2PolygonShape bs2;
        
        b2Vec2 vertices[3];

        vertices[0].Set(pos.x + si*length*cos(theta + deltheta/2.0)+length*sin(theta + deltheta/2.0),
        pos.y + si*length*sin(theta + deltheta/2.0)-length*cos(theta + deltheta/2.0));

        vertices[1].Set(pos.x + (si)*length*cos(theta + deltheta/2.0)+length*sin(theta+deltheta/2.0)+3*length*cos(theta)
          ,pos.y + (si)*length*sin(theta + deltheta/2.0)+length*sin(theta)- length*cos(theta+deltheta/2.0));
        
        vertices[2].Set(pos.x +si*length*cos(theta + deltheta/2.0)-length*sin(theta + deltheta/2.0),
        pos.y + si*length*sin(theta + deltheta/2.0)+length*cos(theta + deltheta/2.0));

        bs2.Set(vertices,3);


        b2FixtureDef *fd3 = new b2FixtureDef;
        fd3->density = 30.0f;
        fd3->shape = &bs2;
        fd3->filter.groupIndex= grpIndex;
        b->CreateFixture(fd3);



        theta=(324+angl)*PI/180;}

        b2RevoluteJointDef revoluteJointDef;
        revoluteJointDef.Initialize(b,reference,b->GetWorldCenter());
        m_world->CreateJoint(&revoluteJointDef);
    
  
      float half_length_shelf = ((centre_x_governer - centre_x_carry_support)/2.0); 


      b2BodyDef bd3;
      bd3.type = b2_dynamicBody;
      bd3.position.Set((centre_x_carry_support+centre_x_governer)/2.0,centre_y_governer+3.00);
      shelf = m_world->CreateBody(&bd3);
            
      b2PolygonShape rect;
      b2Vec2 pos2(0.0f,0.0f);
      rect.SetAsBox(half_length_shelf,0.1f,pos2,0);

      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 30.0f;
      fd3->shape = &rect;
      shelf->CreateFixture(fd3);


      
      
      b2BodyDef bd4;
      bd4.type = b2_dynamicBody;
      bd4.position.Set(((centre_x_carry_support+centre_x_governer)/2.0)-3.00,centre_y_governer);
      shelf2 = m_world->CreateBody(&bd4);
            
      b2PolygonShape rect2;
      b2Vec2 pos3(0.0f,0.0f);
      rect2.SetAsBox(half_length_shelf,0.1f,pos3,0);

      b2FixtureDef *fd4 = new b2FixtureDef;
      fd4->density = 30.0f;
      fd4->shape = &rect2;
      shelf2->CreateFixture(fd4);




      b2RevoluteJointDef revoluteJointDef1;
      revoluteJointDef1.bodyA = shelf;
      revoluteJointDef1.bodyB = gov;
      revoluteJointDef1.collideConnected = false;
      revoluteJointDef1.localAnchorA.Set(half_length_shelf,0.00);
      revoluteJointDef1.localAnchorB.Set(0.00,3.00);
      m_joint =(b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef1 );

      b2RevoluteJointDef revoluteJointDef2;
      revoluteJointDef2.bodyA = shelf;
      revoluteJointDef2.bodyB = b;
      revoluteJointDef2.collideConnected = false;
      revoluteJointDef2.localAnchorA.Set(-half_length_shelf,0.00);
      revoluteJointDef2.localAnchorB.Set(0.00,3.00);
      m_joint2 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef2 );


      b2RevoluteJointDef revoluteJointDef12;
      revoluteJointDef12.bodyA = shelf2;
      revoluteJointDef12.bodyB = gov;
      revoluteJointDef12.collideConnected = false;
      revoluteJointDef12.localAnchorA.Set(half_length_shelf,0.00);
      revoluteJointDef12.localAnchorB.Set(-3.00,0.00);
      m_joint =(b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef12 );


      b2RevoluteJointDef revoluteJointDef22;
      revoluteJointDef22.bodyA = shelf2;
      revoluteJointDef22.bodyB = b;
      revoluteJointDef22.collideConnected = false;
      revoluteJointDef22.localAnchorA.Set(-half_length_shelf,0.00);
      revoluteJointDef22.localAnchorB.Set(-3.00,0.00);
      m_joint2 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef22 );


    return b;
}












  dominos_t::dominos_t()
  {

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
    
      float radius_input = 4.0f;
      float centre_x_input = 35.0f;
      float centre_y_input = 10.0f;
      
      body = generateSpokedWheel(1,radius_input,centre_x_input,centre_y_input,18.0,m_world,reference,10,250.0f);
      
      float radius_governer = 8.0f;
      float centre_x_governer = 30.0f;
      float distance = (radius_governer + radius_input + max(radius_governer,radius_input)*PI/20 + 0.1*min(radius_governer,radius_input));
      float centre_y_governer = sqrt(distance*distance - pow((centre_x_input - centre_x_governer),2)) + centre_y_input; 
      
      float radius_output = 4.0f;
      float centre_x_output = 30.0f;
      float distance2= (radius_governer + radius_output + max(radius_governer,radius_output)*PI/20 + 0.1*min(radius_governer,radius_output));
      float centre_y_output = centre_y_governer + sqrt(distance2*distance2 - pow((centre_x_output - centre_x_governer),2));

      governer = generateSpokedWheel(1,radius_governer,centre_x_governer,centre_y_governer,18.0,m_world,reference,20,50.0f);
      output = generateSpokedWheel(1,radius_output,centre_x_output,centre_y_output,0.0,m_world,reference,10,5.0f);


    { 
    

      b = generateTwoSpokedWheel(governer,m_world,-1);


      float xshift = 48.0f;

    {
      float radius_input = 4.0f;
      float centre_x_input = 35.0f - xshift;
      float centre_y_input = 10.0f ;
      
      body2 = generateSpokedWheel(1,radius_input,centre_x_input,centre_y_input,18.0,m_world,reference,10,45.0f);
      dirchanger = generateSpokedWheel(-1,radius_input-.15,centre_x_governer + 1.7*radius_governer - xshift ,centre_y_governer,0.0,m_world,reference,10,25.0f);
      y_ca = centre_y_governer;
      float radius_governer = 8.0f;
      float centre_x_governer = 30.0f - xshift;
      float distance = (radius_governer + radius_input + max(radius_governer,radius_input)*PI/20 + 0.1*min(radius_governer,radius_input));
      float centre_y_governer = sqrt(distance*distance - pow((centre_x_input - centre_x_governer),2)) + centre_y_input; 
      
      float radius_output = 4.0f;
      float centre_x_output = 30.0f - xshift;
      float distance2= (radius_governer + radius_output + max(radius_governer,radius_output)*PI/20 + 0.1*min(radius_governer,radius_output));
      float centre_y_output = centre_y_governer + sqrt(distance2*distance2 - pow((centre_x_output - centre_x_governer),2));


      governer2 = generateSpokedWheel(1,radius_governer,centre_x_governer,centre_y_governer,36.0,m_world,reference,20,15.0f);
      output2 = generateSpokedWheel(1,radius_output,centre_x_output,centre_y_output,18.0,m_world,reference,10,5.0f);
     
    }




{
       b2Body* hinge;
       float xreq=-34.0f;
       float yreq=24.0f;
       b2BodyDef ref;
       ref.type= b2_staticBody;
       ref.position.Set(xreq,yreq);
       hinge=m_world->CreateBody(&ref);
       b2CircleShape circle;
       circle.m_radius=1.0f;
       b2FixtureDef *fd3 = new b2FixtureDef;
       fd3->shape=&circle;
       hinge->CreateFixture(fd3);

       b2Body* tri; 
       b2BodyDef bd;
       bd.type = b2_dynamicBody; 
       bd.position.Set(xreq,yreq); 
       tri = m_world->CreateBody(&bd); 
       b2PolygonShape shapetemp; 
       b2Vec2 vertices[3]; 
       vertices[0].Set(0,0); 
       vertices[1].Set(9,5); 
       vertices[2].Set(11,7); 
       shapetemp.Set(vertices,3); 
       b2FixtureDef *fd2 = new b2FixtureDef; 
       fd2->density = 1.0f; 
       fd2->shape = &shapetemp; 
       fd2->friction = 0; 
       tri->CreateFixture(fd2);
       b2PolygonShape shapetemp2; 
       b2Vec2 vertices2[3]; 
       vertices2[0].Set(11,7); 
       vertices2[1].Set(9,5); 
       vertices2[2].Set(9.5,4); 
       shapetemp2.Set(vertices2,3); 
       b2FixtureDef *fd = new b2FixtureDef; 
       fd->density = 1.0f; 
       fd->friction = 0; 
       fd->shape = &shapetemp2; 
       tri->CreateFixture(fd); 

       b2RevoluteJointDef stopper;  
       b2Vec2 pos(xreq,yreq);
       stopper.Initialize(hinge,tri,pos); 
       m_world->CreateJoint(&stopper);
     }


    {
       b2Body* hinge;
       b2BodyDef ref;
       ref.type= b2_staticBody;
       ref.position.Set(45.5f,25.0f);
       hinge=m_world->CreateBody(&ref);
       b2CircleShape circle;
       circle.m_radius=1.0f;
       b2FixtureDef *fd3 = new b2FixtureDef;
       
       fd3->shape=&circle;
       hinge->CreateFixture(fd3);

       b2Body* tri; 
       b2BodyDef bd;
       bd.type = b2_dynamicBody; 
       bd.position.Set(45.5f,25.0f); 
       tri = m_world->CreateBody(&bd); 
       b2PolygonShape shapetemp; 
       b2Vec2 vertices[3]; 
       vertices[0].Set(0,0); 
       vertices[1].Set(-10,5); 
       vertices[2].Set(-9,7); 
       shapetemp.Set(vertices,3); 
       b2FixtureDef *fd2 = new b2FixtureDef; 
       fd2->density = 1.0f; 
       fd2->shape = &shapetemp; 
       fd2->friction = 0; 
       tri->CreateFixture(fd2);
       b2PolygonShape shapetemp2; 
       b2Vec2 vertices2[4]; 
       vertices2[0].Set(-9,7); 
       vertices2[1].Set(-11,3); 
       vertices2[2].Set(-12,5);
       vertices2[3].Set(-11,7); 
       shapetemp2.Set(vertices2,4); 
       b2FixtureDef *fd = new b2FixtureDef; 
       fd->density = 150.0f; 
       fd->friction = 0; 
       fd->shape = &shapetemp2; 
       tri->CreateFixture(fd); 

       b2RevoluteJointDef stopper;  
       b2Vec2 pos(45.5f,25.0f);
       stopper.Initialize(hinge,tri,pos); 
       m_world->CreateJoint(&stopper);
     }

  }}
  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
