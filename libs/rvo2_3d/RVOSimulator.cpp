/*
 * RVOSimulator.cpp
 * RVO2-3D Library
 *
 * Copyright 2008 University of North Carolina at Chapel Hill
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/RVO2/>
 */

#include "RVOSimulator.h"

#ifdef _OPENMP
#include <omp.h>
#endif

#include "Agent.h"
#include "KdTree.h"

namespace RVO {
	RVOSimulator::RVOSimulator() : defaultAgent_(NULL), kdTree_(NULL), globalTime_(0.0f), timeStep_(0.0f)
	{
		kdTree_ = new KdTree(this);

		// Load Environment Model
		std::string packagePath = ros::package::getPath("dorca");
		const std::string MODEL_PATH_ENV = packagePath + "/models/env.obj";
	    tinyobj::attrib_t attrib_Env;
    	std::vector<tinyobj::shape_t> shapes_Env;
    	std::vector<tinyobj::material_t> materials_Env;
    	std::string err_Env;
		
		if (!tinyobj::LoadObj(&attrib_Env, &shapes_Env, &materials_Env, &err_Env, MODEL_PATH_ENV.c_str())) {
        	throw std::runtime_error(err_Env);
    	}
		
	    b1->BeginModel();
		std::vector<uint32_t> indices;

        for (const auto& shape : shapes_Env) {
        	int index = 0;
            for (int i = 0; i < shape.mesh.indices.size(); i+=3) {

            	PQP_REAL a[3],b[3],c[3];
            	a[0] = attrib_Env.vertices[3 * shape.mesh.indices[i].vertex_index + 0];
            	a[1] = attrib_Env.vertices[3 * shape.mesh.indices[i].vertex_index + 1];
            	a[2] = attrib_Env.vertices[3 * shape.mesh.indices[i].vertex_index + 2];

            	b[0] = attrib_Env.vertices[3 * shape.mesh.indices[i+1].vertex_index + 0];
            	b[1] = attrib_Env.vertices[3 * shape.mesh.indices[i+1].vertex_index + 1];
            	b[2] = attrib_Env.vertices[3 * shape.mesh.indices[i+1].vertex_index + 2];

            	c[0] = attrib_Env.vertices[3 * shape.mesh.indices[i+2].vertex_index + 0];
            	c[1] = attrib_Env.vertices[3 * shape.mesh.indices[i+2].vertex_index + 1];
            	c[2] = attrib_Env.vertices[3 * shape.mesh.indices[i+2].vertex_index + 2];

            	b1->AddTri(a, b, c, index);
            	index++;
            }
        }
		b1->EndModel();

		//Load Quadrotor Model
		const std::string MODEL_PATH_QUAD = packagePath + "/models/quad.obj";

    	tinyobj::attrib_t attrib_Quad;
    	std::vector<tinyobj::shape_t> shapes_Quad;
    	std::vector<tinyobj::material_t> materials_Quad;
    	std::string err_Quad;

    	if (!tinyobj::LoadObj(&attrib_Quad, &shapes_Quad, &materials_Quad, &err_Quad, MODEL_PATH_QUAD.c_str())) {
        	throw std::runtime_error(err_Quad);
    	}
        b2->BeginModel();

        for (const auto& shape : shapes_Quad) {
        	int index = 0;
            for (int i = 0; i < shape.mesh.indices.size(); i+=3) {
            	PQP_REAL a[3],b[3],c[3];
            	a[0] = attrib_Quad.vertices[3 * shape.mesh.indices[i].vertex_index + 0];
            	a[1] = attrib_Quad.vertices[3 * shape.mesh.indices[i].vertex_index + 1];
            	a[2] = attrib_Quad.vertices[3 * shape.mesh.indices[i].vertex_index + 2];

            	b[0] = attrib_Quad.vertices[3 * shape.mesh.indices[i+1].vertex_index + 0];
            	b[1] = attrib_Quad.vertices[3 * shape.mesh.indices[i+1].vertex_index + 1];
            	b[2] = attrib_Quad.vertices[3 * shape.mesh.indices[i+1].vertex_index + 2];

            	c[0] = attrib_Quad.vertices[3 * shape.mesh.indices[i+2].vertex_index + 0];
            	c[1] = attrib_Quad.vertices[3 * shape.mesh.indices[i+2].vertex_index + 1];
            	c[2] = attrib_Quad.vertices[3 * shape.mesh.indices[i+2].vertex_index + 2];

            	b2->AddTri(a, b, c, index);
            	index++;
            }
        }
		b2->EndModel();
	}

	RVOSimulator::RVOSimulator(float timeStep, float neighborDist, size_t maxNeighbors, float timeHorizon, float radius, float maxSpeed, const Vector3 &velocity) : defaultAgent_(NULL), kdTree_(NULL), globalTime_(0.0f), timeStep_(timeStep)
	{
		kdTree_ = new KdTree(this);
		defaultAgent_ = new Agent(this);

		defaultAgent_->maxNeighbors_ = maxNeighbors;
		defaultAgent_->maxSpeed_ = maxSpeed;
		defaultAgent_->neighborDist_ = neighborDist;
		defaultAgent_->radius_ = radius;
		defaultAgent_->timeHorizon_ = timeHorizon;
		defaultAgent_->velocity_ = velocity;

		// Load Environment Model
		std::string packagePath = ros::package::getPath("dorca");
		const std::string MODEL_PATH_ENV = packagePath + "/models/env.obj";
	    tinyobj::attrib_t attrib_Env;
    	std::vector<tinyobj::shape_t> shapes_Env;
    	std::vector<tinyobj::material_t> materials_Env;
    	std::string err_Env;
		
		if (!tinyobj::LoadObj(&attrib_Env, &shapes_Env, &materials_Env, &err_Env, MODEL_PATH_ENV.c_str())) {
        	throw std::runtime_error(err_Env);
    	}
		
	    b1->BeginModel();
		std::vector<uint32_t> indices;

        for (const auto& shape : shapes_Env) {
        	int index = 0;
            for (int i = 0; i < shape.mesh.indices.size(); i+=3) {

            	PQP_REAL a[3],b[3],c[3];
            	a[0] = attrib_Env.vertices[3 * shape.mesh.indices[i].vertex_index + 0];
            	a[1] = attrib_Env.vertices[3 * shape.mesh.indices[i].vertex_index + 1];
            	a[2] = attrib_Env.vertices[3 * shape.mesh.indices[i].vertex_index + 2];

            	b[0] = attrib_Env.vertices[3 * shape.mesh.indices[i+1].vertex_index + 0];
            	b[1] = attrib_Env.vertices[3 * shape.mesh.indices[i+1].vertex_index + 1];
            	b[2] = attrib_Env.vertices[3 * shape.mesh.indices[i+1].vertex_index + 2];

            	c[0] = attrib_Env.vertices[3 * shape.mesh.indices[i+2].vertex_index + 0];
            	c[1] = attrib_Env.vertices[3 * shape.mesh.indices[i+2].vertex_index + 1];
            	c[2] = attrib_Env.vertices[3 * shape.mesh.indices[i+2].vertex_index + 2];

            	b1->AddTri(a, b, c, index);
            	index++;
            }
        }
		b1->EndModel();

		//Load Quadrotor Model
		const std::string MODEL_PATH_QUAD = packagePath + "/models/quad.obj";

    	tinyobj::attrib_t attrib_Quad;
    	std::vector<tinyobj::shape_t> shapes_Quad;
    	std::vector<tinyobj::material_t> materials_Quad;
    	std::string err_Quad;

    	if (!tinyobj::LoadObj(&attrib_Quad, &shapes_Quad, &materials_Quad, &err_Quad, MODEL_PATH_QUAD.c_str())) {
        	throw std::runtime_error(err_Quad);
    	}
        b2->BeginModel();

        for (const auto& shape : shapes_Quad) {
        	int index = 0;
            for (int i = 0; i < shape.mesh.indices.size(); i+=3) {
            	PQP_REAL a[3],b[3],c[3];
            	a[0] = attrib_Quad.vertices[3 * shape.mesh.indices[i].vertex_index + 0];
            	a[1] = attrib_Quad.vertices[3 * shape.mesh.indices[i].vertex_index + 1];
            	a[2] = attrib_Quad.vertices[3 * shape.mesh.indices[i].vertex_index + 2];

            	b[0] = attrib_Quad.vertices[3 * shape.mesh.indices[i+1].vertex_index + 0];
            	b[1] = attrib_Quad.vertices[3 * shape.mesh.indices[i+1].vertex_index + 1];
            	b[2] = attrib_Quad.vertices[3 * shape.mesh.indices[i+1].vertex_index + 2];

            	c[0] = attrib_Quad.vertices[3 * shape.mesh.indices[i+2].vertex_index + 0];
            	c[1] = attrib_Quad.vertices[3 * shape.mesh.indices[i+2].vertex_index + 1];
            	c[2] = attrib_Quad.vertices[3 * shape.mesh.indices[i+2].vertex_index + 2];

            	b2->AddTri(a, b, c, index);
            	index++;
            }
        }
		b2->EndModel();
	}

	RVOSimulator::~RVOSimulator()
	{
		if (defaultAgent_ != NULL) {
			delete defaultAgent_;
		}

		for (size_t i = 0; i < agents_.size(); ++i) {
			delete agents_[i];
		}

		if (kdTree_ != NULL) {
			delete kdTree_;
		}
	}

	size_t RVOSimulator::getAgentNumAgentNeighbors(size_t agentNo) const
	{
		return agents_[agentNo]->agentNeighbors_.size();
	}

	size_t RVOSimulator::getAgentAgentNeighbor(size_t agentNo, size_t neighborNo) const
	{
		return agents_[agentNo]->agentNeighbors_[neighborNo].second->id_;
	}

	size_t RVOSimulator::getAgentNumORCAPlanes(size_t agentNo) const
	{
		return agents_[agentNo]->orcaPlanes_.size();
	}

	const Plane &RVOSimulator::getAgentORCAPlane(size_t agentNo, size_t planeNo) const
	{
		return agents_[agentNo]->orcaPlanes_[planeNo];
	}

	void RVOSimulator::removeAgent(size_t agentNo)
	{
		delete agents_[agentNo];
		agents_[agentNo] = agents_.back();
		agents_.pop_back();
	}

	size_t RVOSimulator::addAgent(const Vector3 &position)
	{
		if (defaultAgent_ == NULL) {
			return RVO_ERROR;
		}

		Agent *agent = new Agent(this);

		agent->position_ = position;
		agent->maxNeighbors_ = defaultAgent_->maxNeighbors_;
		agent->maxSpeed_ = defaultAgent_->maxSpeed_;
		agent->neighborDist_ = defaultAgent_->neighborDist_;
		agent->radius_ = defaultAgent_->radius_;
		agent->timeHorizon_ = defaultAgent_->timeHorizon_;
		agent->velocity_ = defaultAgent_->velocity_;

		agent->id_ = agents_.size();

		agents_.push_back(agent);

		return agents_.size() - 1;
	}

	size_t RVOSimulator::addAgent(const Vector3 &position, float neighborDist, size_t maxNeighbors, float timeHorizon, float radius, float maxSpeed, const Vector3 &velocity)
	{
		Agent *agent = new Agent(this);

		agent->position_ = position;
		agent->maxNeighbors_ = maxNeighbors;
		agent->maxSpeed_ = maxSpeed;
		agent->neighborDist_ = neighborDist;
		agent->radius_ = radius;
		agent->timeHorizon_ = timeHorizon;
		agent->velocity_ = velocity;

		agent->id_ = agents_.size();

		agents_.push_back(agent);

		return agents_.size() - 1;
	}

	void RVOSimulator::doStep()
	{
		kdTree_->buildAgentTree();

#ifdef _OPENMP
#pragma omp parallel for
#endif
		for (int i = 0; i < static_cast<int>(agents_.size()); ++i) {
			agents_[i]->computeNeighbors();
			agents_[i]->computeNewVelocity(b1, b2);
		}

#ifdef _OPENMP
#pragma omp parallel for
#endif
		for (int i = 0; i < static_cast<int>(agents_.size()); ++i) {
			agents_[i]->update();
		}

		globalTime_ += timeStep_;
	}

	size_t RVOSimulator::getAgentMaxNeighbors(size_t agentNo) const
	{
		return agents_[agentNo]->maxNeighbors_;
	}

	float RVOSimulator::getAgentMaxSpeed(size_t agentNo) const
	{
		return agents_[agentNo]->maxSpeed_;
	}

	float RVOSimulator::getAgentNeighborDist(size_t agentNo) const
	{
		return agents_[agentNo]->neighborDist_;
	}

	const Vector3 &RVOSimulator::getAgentPosition(size_t agentNo) const
	{
		return agents_[agentNo]->position_;
	}

	const Vector3 &RVOSimulator::getAgentPrefVelocity(size_t agentNo) const
	{
		return agents_[agentNo]->prefVelocity_;
	}

	float RVOSimulator::getAgentRadius(size_t agentNo) const
	{
		return agents_[agentNo]->radius_;
	}

	float RVOSimulator::getAgentTimeHorizon(size_t agentNo) const
	{
		return agents_[agentNo]->timeHorizon_;
	}

	const Vector3 &RVOSimulator::getAgentVelocity(size_t agentNo) const
	{
		return agents_[agentNo]->velocity_;
	}

	float RVOSimulator::getGlobalTime() const
	{
		return globalTime_;
	}

	size_t RVOSimulator::getNumAgents() const
	{
		return agents_.size();
	}

	float RVOSimulator::getTimeStep() const
	{
		return timeStep_;
	}

	void RVOSimulator::setAgentDefaults(float neighborDist, size_t maxNeighbors, float timeHorizon, float radius, float maxSpeed, const Vector3 &velocity)
	{
		if (defaultAgent_ == NULL) {
			defaultAgent_ = new Agent(this);
		}

		defaultAgent_->maxNeighbors_ = maxNeighbors;
		defaultAgent_->maxSpeed_ = maxSpeed;
		defaultAgent_->neighborDist_ = neighborDist;
		defaultAgent_->radius_ = radius;
		defaultAgent_->timeHorizon_ = timeHorizon;
		defaultAgent_->velocity_ = velocity;
	}

	void RVOSimulator::setAgentMaxNeighbors(size_t agentNo, size_t maxNeighbors)
	{
		agents_[agentNo]->maxNeighbors_ = maxNeighbors;
	}

	void RVOSimulator::setAgentMaxSpeed(size_t agentNo, float maxSpeed)
	{
		agents_[agentNo]->maxSpeed_ = maxSpeed;
	}

	void RVOSimulator::setAgentNeighborDist(size_t agentNo, float neighborDist)
	{
		agents_[agentNo]->neighborDist_ = neighborDist;
	}

	void RVOSimulator::setAgentPosition(size_t agentNo, const Vector3 &position)
	{
		agents_[agentNo]->position_ = position;
	}

	void RVOSimulator::setAgentPrefVelocity(size_t agentNo, const Vector3 &prefVelocity)
	{
		agents_[agentNo]->prefVelocity_ = prefVelocity;
	}

	void RVOSimulator::setAgentRadius(size_t agentNo, float radius)
	{
		agents_[agentNo]->radius_ = radius;
	}

	void RVOSimulator::setAgentTimeHorizon(size_t agentNo, float timeHorizon)
	{
		agents_[agentNo]->timeHorizon_ = timeHorizon;
	}

	void RVOSimulator::setAgentVelocity(size_t agentNo, const Vector3 &velocity)
	{
		agents_[agentNo]->velocity_ = velocity;
	}

	void RVOSimulator::setTimeStep(float timeStep)
	{
		timeStep_ = timeStep;
	}
}
