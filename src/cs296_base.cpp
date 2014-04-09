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

#include "cs296_base.hpp"
#include <cstdio>
#include <stdio.h>
#include <string>
#include <stdlib.h>
#include <iostream>
#include "dominos.hpp"
#include <math.h>
#define DEGTORAD 0.0174532925199432957f
#define PI 3.14159

using namespace std;
using namespace cs296;

extern b2Body *body;
extern int initialAngle;
extern int finalAngle;
extern int initialAngle2;
extern int finalAngle2;
extern b2Body *governer ;
extern b2Body *governer2 ;
extern b2Body *body2 ;
extern b2Body *output ;
extern b2Body *output2 ;

extern b2Body* reference;
extern b2World* m_world;
extern float y_ca;
extern b2Body* b;
extern b2Body* dirchanger;


extern b2Body* shelf;
extern b2Body* shelf2;


int input = 0;
int input2 = 0;

bool firstTime = true;
bool firstTime2 = true;



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

      //b2Body* shelf;
      

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


      //b2Body* shelf2;
      
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
      (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef1 );

      b2RevoluteJointDef revoluteJointDef2;
      revoluteJointDef2.bodyA = shelf;
      revoluteJointDef2.bodyB = b;
      revoluteJointDef2.collideConnected = false;
      revoluteJointDef2.localAnchorA.Set(-half_length_shelf,0.00);
      revoluteJointDef2.localAnchorB.Set(0.00,3.00);
      (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef2 );


      b2RevoluteJointDef revoluteJointDef12;
      revoluteJointDef12.bodyA = shelf2;
      revoluteJointDef12.bodyB = gov;
      revoluteJointDef12.collideConnected = false;
      revoluteJointDef12.localAnchorA.Set(half_length_shelf,0.00);
      revoluteJointDef12.localAnchorB.Set(-3.00,0.00);
      (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef12 );


      b2RevoluteJointDef revoluteJointDef22;
      revoluteJointDef22.bodyA = shelf2;
      revoluteJointDef22.bodyB = b;
      revoluteJointDef22.collideConnected = false;
      revoluteJointDef22.localAnchorA.Set(-half_length_shelf,0.00);
      revoluteJointDef22.localAnchorB.Set(-3.00,0.00);
      (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef22 );


    return b;
}





base_sim_t::base_sim_t()
{
	b2Vec2 gravity;
	gravity.Set(0.0f, -10.0f);
	m_world = new b2World(gravity);

	m_text_line = 30;

	m_point_count = 0;

	m_world->SetDebugDraw(&m_debug_draw);
	
	m_step_count = 0;

	b2BodyDef body_def;
	m_ground_body = m_world->CreateBody(&body_def);

	memset(&m_max_profile, 0, sizeof(b2Profile));
	memset(&m_total_profile, 0, sizeof(b2Profile));
}

base_sim_t::~base_sim_t()
{
	// By deleting the world, we delete the bomb, mouse joint, etc.
	delete m_world;
	m_world = NULL;
}

void base_sim_t::pre_solve(b2Contact* contact, const b2Manifold* oldManifold)
{
  const b2Manifold* manifold = contact->GetManifold();
  
  if (manifold->pointCount == 0)
    {
      return;
    }
  
  b2Fixture* fixtureA = contact->GetFixtureA();
  b2Fixture* fixtureB = contact->GetFixtureB();
  
  b2PointState state1[b2_maxManifoldPoints], state2[b2_maxManifoldPoints];
  b2GetPointStates(state1, state2, oldManifold, manifold);
  
  b2WorldManifold world_manifold;
  contact->GetWorldManifold(&world_manifold);
  
  for (int32 i = 0; i < manifold->pointCount && m_point_count < k_max_contact_points; ++i)
    {
      contact_point_t* cp = m_points + m_point_count;
      cp->fixtureA = fixtureA;
      cp->fixtureB = fixtureB;
      cp->position = world_manifold.points[i];
      cp->normal = world_manifold.normal;
      cp->state = state2[i];
      ++m_point_count;
    }
}

void base_sim_t::draw_title(int x, int y, const char *string)
{
    m_debug_draw.DrawString(x, y, string);
}

void base_sim_t::step(settings_t* settings)
{

 
  char numero = int((output->GetAngle()*180/PI+25)/36)%10 + '0';
  m_debug_draw.DrawString(900,30,"Output units digit");
  m_debug_draw.DrawString(1050,30,&numero);
  char numero2 = int((output2->GetAngle()*180/PI+25)/36)%10 +'0';
  m_debug_draw.DrawString(450,30,"Output tens digit");
  m_debug_draw.DrawString(600,30,&numero2);
  
  
  char numero3 = (input/36)%10 + '0';
  m_debug_draw.DrawString(900,630,"Input Units digit");
  m_debug_draw.DrawString(1050,630,&numero3);
  
  
  char numero4 = (input2/36)%10 + '0';
  m_debug_draw.DrawString(450,630,"Input Tens digit");
  m_debug_draw.DrawString(600,630,&numero4);
  
 
  if(finalAngle2 > initialAngle2)
  {
    if(firstTime2)
      {
        float b_angle = b->GetAngle();
      
      b2Vec2 pos = dirchanger->GetPosition();
      float angle = dirchanger->GetAngle();
      
      m_world->DestroyBody(dirchanger);
      m_world->DestroyBody(b);
      m_world->DestroyBody(shelf);
      m_world->DestroyBody(shelf2);
      b= generateTwoSpokedWheel(governer,m_world,-1);
      b2Vec2 vec;
      vec.Set(0.0,0.0);
      b->SetTransform(vec,b_angle);

      dirchanger = generateSpokedWheel(-1, 3.85,pos.x,pos.y,angle*180/PI,m_world,reference,10,25.0f);
      firstTime2 = false;
      firstTime = true;
      }
    
      input2 +=2;
      initialAngle2 +=2;
      body2->SetTransform( body2->GetPosition(), initialAngle2*DEGTORAD );      
  /*
      if(finalAngle==initialAngle)
      {
        m_world->DestroyBody(dirchanger);
        m_world->DestroyBody(governer2);
        dirchanger = generateSpokedWheel(-1,4.0f,30.0f + 1.7*8.0f - 48.0f ,y_ca,0.0,m_world,reference,10,1.0f);
        governer2 = generateSpokedWheel(-1,radius_governer,centre_x_governer,centre_y_governer,0.0,m_world,reference,20,0.5f);
      } */ 
  }  
  
  

  if(finalAngle2 == initialAngle2 )
  {
    if(finalAngle > initialAngle)
    {
      if(firstTime)
      {
        float b_angle = b->GetAngle();
      
      b2Vec2 pos = dirchanger->GetPosition();
      float angle = dirchanger->GetAngle();
      
      m_world->DestroyBody(dirchanger);
      m_world->DestroyBody(b);
      m_world->DestroyBody(shelf);
       m_world->DestroyBody(shelf2);
      
      b= generateTwoSpokedWheel(governer,m_world,1);
      b2Vec2 vec;
      vec.Set(0.0,0.0);
      b->SetTransform(vec,b_angle);

      dirchanger = generateSpokedWheel(1, 3.85,pos.x,pos.y,angle*180/PI,m_world,reference,10,25.0f);
      firstTime = false;
      firstTime2 = true;
      
      }
      
      input+=2;
      initialAngle +=2;
      body->SetTransform( body->GetPosition(), initialAngle*DEGTORAD );    
      
    }

  }
  // if(finalAngle2 > initialAngle2)
  // {
  //   input2+=2;
  //     initialAngle2 +=2;
  //     body2->SetTransform( body2->GetPosition(), initialAngle2*DEGTORAD );      
  // }  
  
  // if(finalAngle2 < initialAngle2)
  // {
  //     input2 -=2;
  //     initialAngle2 -=2;
  //     body->SetTransform( body2->GetPosition(), initialAngle2*DEGTORAD );      
  // }

  if(finalAngle==initialAngle && finalAngle2==initialAngle2)
  {
      m_debug_draw.DrawString(200,100,"Enter Next Number");
      input = 0;
      input2 = 0;
  }



  float32 time_step = settings->hz > 0.0f ? 1.0f / settings->hz : float32(0.0f);

  if (settings->pause)
    {
      if (settings->single_step)
	{
	  settings->single_step = 0;
	}
      else
	{
	  time_step = 0.0f;
	}
      
      m_debug_draw.DrawString(5, m_text_line, "****PAUSED****");
      m_text_line += 15;
    }
  
  uint32 flags = 0;
  flags += settings->draw_shapes			* b2Draw::e_shapeBit;
  flags += settings->draw_joints			* b2Draw::e_jointBit;
  flags += settings->draw_AABBs			* b2Draw::e_aabbBit;
  flags += settings->draw_pairs			* b2Draw::e_pairBit;
  flags += settings->draw_COMs				* b2Draw::e_centerOfMassBit;
  m_debug_draw.SetFlags(flags);
  
  m_world->SetWarmStarting(settings->enable_warm_starting > 0);
  m_world->SetContinuousPhysics(settings->enable_continuous > 0);
  m_world->SetSubStepping(settings->enable_sub_stepping > 0);
  
  m_point_count = 0;
  
  m_world->Step(time_step, settings->velocity_iterations, settings->position_iterations);
  
  m_world->DrawDebugData();
  
  if (time_step > 0.0f)
    {
      ++m_step_count;
    }
  
  if (settings->draw_stats)
    {
      int32 body_count = m_world->GetBodyCount();
      int32 contact_count = m_world->GetContactCount();
      int32 joint_count = m_world->GetJointCount();
      m_debug_draw.DrawString(5, m_text_line, "bodies/contacts/joints = %d/%d/%d", body_count, contact_count, joint_count);
      m_text_line += 15;
      
      int32 proxy_count = m_world->GetProxyCount();
      int32 height = m_world->GetTreeHeight();
      int32 balance = m_world->GetTreeBalance();
      float32 quality = m_world->GetTreeQuality();
      m_debug_draw.DrawString(5, m_text_line, "proxies/height/balance/quality = %d/%d/%d/%g", proxy_count, height, balance, quality);
      m_text_line += 15;
    }
  
  // Track maximum profile times
  {
    const b2Profile& p = m_world->GetProfile();
    m_max_profile.step = b2Max(m_max_profile.step, p.step);
    m_max_profile.collide = b2Max(m_max_profile.collide, p.collide);
    m_max_profile.solve = b2Max(m_max_profile.solve, p.solve);
    m_max_profile.solveInit = b2Max(m_max_profile.solveInit, p.solveInit);
    m_max_profile.solveVelocity = b2Max(m_max_profile.solveVelocity, p.solveVelocity);
    m_max_profile.solvePosition = b2Max(m_max_profile.solvePosition, p.solvePosition);
    m_max_profile.solveTOI = b2Max(m_max_profile.solveTOI, p.solveTOI);
    m_max_profile.broadphase = b2Max(m_max_profile.broadphase, p.broadphase);
    
    m_total_profile.step += p.step;
    m_total_profile.collide += p.collide;
    m_total_profile.solve += p.solve;
    m_total_profile.solveInit += p.solveInit;
    m_total_profile.solveVelocity += p.solveVelocity;
    m_total_profile.solvePosition += p.solvePosition;
    m_total_profile.solveTOI += p.solveTOI;
    m_total_profile.broadphase += p.broadphase;
  }
  
  if (settings->draw_profile)
    {
      const b2Profile& p = m_world->GetProfile();
      
      b2Profile ave_profile;
      memset(&ave_profile, 0, sizeof(b2Profile));
      if (m_step_count > 0)
	{
	  float32 scale = 1.0f / m_step_count;
	  ave_profile.step = scale * m_total_profile.step;
	  ave_profile.collide = scale * m_total_profile.collide;
	  ave_profile.solve = scale * m_total_profile.solve;
	  ave_profile.solveInit = scale * m_total_profile.solveInit;
	  ave_profile.solveVelocity = scale * m_total_profile.solveVelocity;
	  ave_profile.solvePosition = scale * m_total_profile.solvePosition;
	  ave_profile.solveTOI = scale * m_total_profile.solveTOI;
	  ave_profile.broadphase = scale * m_total_profile.broadphase;
	}
      
      m_debug_draw.DrawString(5, m_text_line, "step [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.step, ave_profile.step, m_max_profile.step);
      m_text_line += 15;
      m_debug_draw.DrawString(5, m_text_line, "collide [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.collide, ave_profile.collide, m_max_profile.collide);
      m_text_line += 15;
      m_debug_draw.DrawString(5, m_text_line, "solve [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solve, ave_profile.solve, m_max_profile.solve);
      m_text_line += 15;
      m_debug_draw.DrawString(5, m_text_line, "solve init [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solveInit, ave_profile.solveInit, m_max_profile.solveInit);
      m_text_line += 15;
      m_debug_draw.DrawString(5, m_text_line, "solve velocity [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solveVelocity, ave_profile.solveVelocity, m_max_profile.solveVelocity);
      m_text_line += 15;
      m_debug_draw.DrawString(5, m_text_line, "solve position [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solvePosition, ave_profile.solvePosition, m_max_profile.solvePosition);
      m_text_line += 15;
      m_debug_draw.DrawString(5, m_text_line, "solveTOI [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solveTOI, ave_profile.solveTOI, m_max_profile.solveTOI);
      m_text_line += 15;
      m_debug_draw.DrawString(5, m_text_line, "broad-phase [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.broadphase, ave_profile.broadphase, m_max_profile.broadphase);
      m_text_line += 15;
    }
    
  if (settings->draw_contact_points)
    {
      //const float32 k_impulseScale = 0.1f;
      const float32 k_axis_scale = 0.3f;
      
      for (int32 i = 0; i < m_point_count; ++i)
	{
	  contact_point_t* point = m_points + i;
	  
	  if (point->state == b2_addState)
	    {
	      // Add
	      m_debug_draw.DrawPoint(point->position, 10.0f, b2Color(0.3f, 0.95f, 0.3f));
	    }
	  else if (point->state == b2_persistState)
	    {
	      // Persist
	      m_debug_draw.DrawPoint(point->position, 5.0f, b2Color(0.3f, 0.3f, 0.95f));
	    }
	  
	  if (settings->draw_contact_normals == 1)
	    {
	      b2Vec2 p1 = point->position;
	      b2Vec2 p2 = p1 + k_axis_scale * point->normal;
	      m_debug_draw.DrawSegment(p1, p2, b2Color(0.9f, 0.9f, 0.9f));
	    }
	  else if (settings->draw_contact_forces == 1)
	    {
	      //b2Vec2 p1 = point->position;
	      //b2Vec2 p2 = p1 + k_forceScale * point->normalForce * point->normal;
	      //DrawSegment(p1, p2, b2Color(0.9f, 0.9f, 0.3f));
	    }
	  
	  if (settings->draw_friction_forces == 1)
	    {
	      //b2Vec2 tangent = b2Cross(point->normal, 1.0f);
	      //b2Vec2 p1 = point->position;
	      //b2Vec2 p2 = p1 + k_forceScale * point->tangentForce * tangent;
	      //DrawSegment(p1, p2, b2Color(0.9f, 0.9f, 0.3f));
	    }
	}
    }
}
