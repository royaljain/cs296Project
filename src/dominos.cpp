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
float radius_inner = 5.0f; 
float centre_x = 14.0f; 
float centre_y = 16.0f;

b2Body* circle_facing ;
float radius_inner_facing = 7.0f; 
float centre_x_facing = -15.0f; 
float centre_y_facing = 30.0f;

float radius_inner_facing2= 11.0f; 
float radius_inner_facing3= 15.0f; 


namespace cs296
{
  /**  The is the constructor 
   * This is the documentation block for the constructor.
   */ 
  
  dominos_t::dominos_t()
  {

    { 
      b2BodyDef bd;
      bd.position.Set(centre_x,centre_y);
      bd.type = b2_dynamicBody;
      body = m_world->CreateBody(&bd);
      
      b2CircleShape circle;
      //circle.m_p.Set(0.0f, 0.0f);
      circle.m_radius = radius_inner;

      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.0f;
      fd->shape = &circle;
      body->CreateFixture(fd);



      for(int i=0;i<10;i++)
      {
        float theta = (36*i-13.5)*PI/180;
        float length = 3.0f;
        float deltheta = 10*PI/180;
        float end = length + radius_inner;
     
        b2PolygonShape bs1;
        b2Vec2 vertices[4];

        vertices[0].Set(radius_inner*cos(theta),radius_inner*sin(theta));
        vertices[1].Set(end*cos(theta),end*sin(theta));
        vertices[2].Set(end*cos(theta - deltheta),end*sin(theta-deltheta));
        vertices[3].Set(radius_inner*cos(theta-deltheta),radius_inner*sin(theta-deltheta));
        
        bs1.Set(vertices,4);


        b2FixtureDef *fd2 = new b2FixtureDef;
        fd2->density = 1.0f;
        fd2->shape = &bs1;
      
        body->CreateFixture(fd2); 
    
      }
    }  

    {  
      b2BodyDef bd;
      bd.position.Set(centre_x_facing,centre_y_facing);
      bd.type = b2_dynamicBody;
      circle_facing = m_world->CreateBody(&bd);
      
      b2CircleShape circle;
      circle.m_radius = radius_inner_facing;

      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.0f;
      fd->shape = &circle;
      circle_facing->CreateFixture(fd);


            

      for(int i=0;i<10;i++)
      {
        float theta = (36*i-13.5)*PI/180;
        float length = radius_inner_facing - 1;
        
     
        b2CircleShape bs1;
        bs1.m_p.Set(length*cos(theta),length*sin(theta));
        bs1.m_radius = 0.75 ;        


        b2FixtureDef *fd2 = new b2FixtureDef;
        fd2->density = 1.0f;
        fd2->shape = &bs1;
      
        circle_facing->CreateFixture(fd2); 
    
      }
      

   
      }   


      {  
      b2BodyDef bd;
      bd.position.Set(centre_x_facing,centre_y_facing);
      bd.type = b2_dynamicBody;
      circle_facing = m_world->CreateBody(&bd);
      
      b2CircleShape circle;
      circle.m_radius = radius_inner_facing2;

      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.0f;
      fd->shape = &circle;
      circle_facing->CreateFixture(fd);


            

      for(int i=0;i<10;i++)
      {
        float theta = (36*i-13.5)*PI/180;
        float length = radius_inner_facing2 - 1;
        
     
        b2CircleShape bs1;
        bs1.m_p.Set(length*cos(theta),length*sin(theta));
        bs1.m_radius = 0.75 ;        


        b2FixtureDef *fd2 = new b2FixtureDef;
        fd2->density = 1.0f;
        fd2->shape = &bs1;
      
        circle_facing->CreateFixture(fd2); 
    
      }      

      } 


    
      b2BodyDef bd;
      bd.position.Set(centre_x,centre_y);
      bd.type = b2_dynamicBody;
      body = m_world->CreateBody(&bd);
      
      b2CircleShape circle;
      //circle.m_p.Set(0.0f, 0.0f);
      circle.m_radius = radius_inner_facing3;

      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.0f;
      fd->shape = &circle;
      circle_facing->CreateFixture(fd);



      for(int i=0;i<10;i++)
      {
        float theta = (36*i-13.5)*PI/180;
        float length = 3.0f;
        float deltheta = 10*PI/180;
        float end = length + radius_inner_facing3;
     
        b2PolygonShape bs1;
        b2Vec2 vertices[4];

        vertices[0].Set(radius_inner*cos(theta),radius_inner*sin(theta));
        vertices[1].Set(end*cos(theta),end*sin(theta));
        vertices[2].Set(end*cos(theta - deltheta),end*sin(theta-deltheta));
        vertices[3].Set(radius_inner*cos(theta-deltheta),radius_inner*sin(theta-deltheta));
        
        bs1.Set(vertices,4);


        b2FixtureDef *fd2 = new b2FixtureDef;
        fd2->density = 1.0f;
        fd2->shape = &bs1;
      
        circle_facing->CreateFixture(fd2); 
    
      }
      


    }


  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
