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

float radius_input = 4.0f;
float centre_x_input = 35.0f;
float centre_y_input = 10.0f;      
float radius_governer = 8.0f;
float centre_x_governer = 30.0f;
float distance3 = (radius_governer + radius_input + max(radius_governer,radius_input)*PI/20 + 0.1*min(radius_governer,radius_input));
float centre_y_governer = sqrt(distance3*distance3 - pow((centre_x_input - centre_x_governer),2)) + centre_y_input;       
float radius_output = 4.0f;
float centre_x_output = 30.0f;
float distance2= (radius_governer + radius_output + max(radius_governer,radius_output)*PI/20 + 0.1*min(radius_governer,radius_output));
float centre_y_output = centre_y_governer + sqrt(distance2*distance2 - pow((centre_x_output - centre_x_governer),2));

b2Body* reference;
b2World* m_world;
b2Body* body ;
b2Body* dirchanger;
b2Body* output;
b2Body* governer; 
b2Body* carry_support; 
b2RevoluteJoint* m_joint;
b2RevoluteJoint* m_joint2;
b2Body* twoSpokes;
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
  
/** \par Generate Spoked Wheel Function */
/** 
* Function generateSpokedWheel :: Parameters = int,float,float,float,float,b2World*,b2Body*,int,float :: Action = creates a spoked wheel 
* given the group index (i.e. bodies collide when they have the same positive group index and don't if they have the same negative group index, radius, the coordinates of its centre, initial angle, world in which it is, body to which it is linked, 
* number of spokes and its density <br>
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

/** \par Generate Two Spoked Wheel Function */
/** 
* Function generateTwoSpokedWheel :: Parameters = b2Body*,b2World*,int :: Action = creates a two spoked wheel 
* given the body to which it is linked, world in which it is and group index. Also, 2 shelves are created which 
 link the spoked wheel to the governer which causes the 2 spoked wheel to rotate the same angle as the governer.
 The radius of the two spoked wheel and the coordinates. 
* of its center are hard coded inside the function <br>
*/


void createRevJoint(b2Body* A,b2Body* B,float Ax,float Ay,float Bx,float By,b2World* m_world )
{

  b2RevoluteJointDef revoluteJointDef;
  revoluteJointDef.bodyA = A;
  revoluteJointDef.bodyB = B;
  revoluteJointDef.collideConnected = false;
  revoluteJointDef.localAnchorA.Set(Ax,Ay);
  revoluteJointDef.localAnchorB.Set(Bx,By);
  m_world->CreateJoint(&revoluteJointDef);	
}	


b2Body* generateTwoSpokedWheel(b2Body* gov,b2World* m_world,int grpIndex)
{
      
      b2Body* b;

      float centre_x_carry_support = 8.4f;
       
      b2BodyDef bd;
      bd.position.Set(centre_x_carry_support,centre_y_governer);
      bd.type = b2_dynamicBody;      
      
      b = m_world->CreateBody(&bd);    

      b2CircleShape circle;
      circle.m_radius = radius_input;

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
      float length = radius_input*sin(deltheta/2.0);
      int si = 4.8;

      for(int i=0;i<2;i++){

        b2PolygonShape bs1;
        b2Vec2 pos((radius_input*cos(deltheta/2.0)+length)*cos(theta + deltheta/2.0),
        (radius_input*cos(deltheta/2.0)+length)*sin(theta + deltheta/2.0));
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

	  createRevJoint(shelf,gov,half_length_shelf,0.00,0.00,3.00,m_world );
	  createRevJoint(shelf,b,-half_length_shelf,0.00,0.00,3.00,m_world );
	  createRevJoint(shelf2,gov,half_length_shelf,0.00,-3.00,0.00,m_world );
	  createRevJoint(shelf2,b,-half_length_shelf,0.00,-3.00,0.00,m_world );
	

    return b;
}

  /** \par Creating the structure of the pascaline */ 
  /** 
  * This creates the basic structure of the pascaline containing 7 spoked wheels and 1 two spoked wheel <br>
  * Variable body :: Type = b2Body* :: Action = this takes the input of the units digit from the user <br>
  * Variable governer :: Type = b2Body* :: Action = this is also used for rotating output and have the hinges attached which 
  prevent backward rotation <br>
  * Variable output :: Type = b2Body* :: Action = this displays the output of the units digit of the answer <br>
  * Variable b :: Type = b2Body* :: Action = this performs the action of carry. It is the only two spoked wheel in the system. 
  * It is connected to the governer. It rotates governer2 after every 10 increments in body <br>
  * Variable body2 :: Type = b2Body* :: Action = this takes the input of the tens digit from the user <br>
  * Variable dirchanger :: Type = b2Body* :: Action = this changes the direction of rotation of the governer2 keeping it the same as the governer <br>
  * Variable governer2 :: Type = b2Body* :: Action = this is also used for rotating output2 and have the hinges attached which 
  prevent backward rotation <br>
  * Variable output2 :: Type = b2Body* :: Action = this displays the output of the tens digit of the answer <br>
  */
  dominos_t::dominos_t()
  {

      b2BodyDef bd;
      bd.type = b2_staticBody;
      bd.position.Set(0.0f,0.0f);
      reference = m_world->CreateBody(&bd);
      
      b2CircleShape circle2;
      circle2.m_radius = 0.01f;

      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 100000.0f;
      fd2->shape = &circle2;
      reference->CreateFixture(fd2);
    
      body = generateSpokedWheel(1,radius_input,centre_x_input,centre_y_input,18.0,m_world,reference,10,250.0f);
      governer = generateSpokedWheel(1,radius_governer,centre_x_governer,centre_y_governer,18.0,m_world,reference,20,50.0f);
      output = generateSpokedWheel(1,radius_output,centre_x_output,centre_y_output,0.0,m_world,reference,10,5.0f);

    {  
      twoSpokes = generateTwoSpokedWheel(governer,m_world,-1);
	  float xshift = 48.0f;
    {
      float radius_input = 4.0f;
      float centre_x_input = 35.0f - xshift;
      float centre_y_input = 10.0f ;
      
      body2 = generateSpokedWheel(1,radius_input,centre_x_input,centre_y_input,18.0,m_world,reference,10,45.0f);
      dirchanger = generateSpokedWheel(-1,radius_input-.15,centre_x_governer + 1.7*radius_governer - xshift ,centre_y_governer,0.0,m_world,reference,10,25.0f);
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



/**
* Variable hinge :: Type = b2Body* :: Action = prevents the reverse movement of the spoked wheels <br>
*/
{
       b2Body* hinge;
       b2BodyDef ref;
       ref.type= b2_staticBody;
       ref.position.Set(45.5f,25.0f);
       hinge=m_world->CreateBody(&ref);
       b2CircleShape circle;
       circle.m_radius=0.01f;
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
       vertices[1].Set(-8.8,4.2); 
       vertices[2].Set(-7.8,6.2); 
       shapetemp.Set(vertices,3); 
       b2FixtureDef *fd2 = new b2FixtureDef; 
       fd2->density = 1000.0f; 
       fd2->shape = &shapetemp; 
       fd2->friction = 0; 
       tri->CreateFixture(fd2);
       b2PolygonShape shapetemp2; 
       b2Vec2 vertices2[4]; 
       vertices2[0].Set(-7.8,6.2); 
       vertices2[1].Set(-10,2); 
       vertices2[2].Set(-11,4);
       vertices2[3].Set(-10.5,6); 
       shapetemp2.Set(vertices2,4); 
       b2FixtureDef *fd = new b2FixtureDef; 
       fd->density = 1000.0f; 
       fd->friction = 0; 
       fd->shape = &shapetemp2; 
       tri->CreateFixture(fd); 

       b2RevoluteJointDef stopper;  
       b2Vec2 pos(45.5f,25.0f);
       stopper.Initialize(hinge,tri,pos); 
       m_world->CreateJoint(&stopper);
     }
  /**
  * Variable hinge :: Type = b2Body* :: Action = prevents the reverse movement of the spoked wheels <br>
  */
  {
       b2Body* hinge;
       b2BodyDef ref;
       ref.type= b2_staticBody;
       ref.position.Set(47.5f-xshift,29.0f);
       hinge=m_world->CreateBody(&ref);
       b2CircleShape circle;
       circle.m_radius=0.01f;
       b2FixtureDef *fd3 = new b2FixtureDef;
       
       fd3->shape=&circle;
       hinge->CreateFixture(fd3);

       b2Body* tri; 
       b2BodyDef bd;
       bd.type = b2_dynamicBody; 
       bd.position.Set(47.5f-xshift,29.0f); 
       tri = m_world->CreateBody(&bd); 
       b2PolygonShape shapetemp; 
       b2Vec2 vertices[3]; 
       vertices[0].Set(0,0); 
       vertices[1].Set(-9,4); 
       vertices[2].Set(-6.5,6); 
       shapetemp.Set(vertices,3); 
       b2FixtureDef *fd2 = new b2FixtureDef; 
       fd2->density = 30.0f; 
       fd2->shape = &shapetemp; 
       fd2->friction = 0; 
       tri->CreateFixture(fd2);
       b2PolygonShape shapetemp2; 
       b2Vec2 vertices2[4]; 
       vertices2[0].Set(-6.5,6); 
       vertices2[1].Set(-11.5,2); 
       vertices2[2].Set(-12.5,4);
       vertices2[3].Set(-7.8,7); 
       shapetemp2.Set(vertices2,4); 
       b2FixtureDef *fd = new b2FixtureDef; 
       fd->density = 30.0f; 
       fd->friction = 0; 
       fd->shape = &shapetemp2; 
       tri->CreateFixture(fd); 

       b2RevoluteJointDef stopper;  
       b2Vec2 pos(47.5f-xshift,29.0f);
       stopper.Initialize(hinge,tri,pos); 
       m_world->CreateJoint(&stopper);
     }
    
  }}
  sim_t *sim = new sim_t("Pascaline", dominos_t::create);
}
