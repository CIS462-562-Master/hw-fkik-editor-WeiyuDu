#include "aActor.h"

#pragma warning(disable : 4018)



/****************************************************************
*
*    	    Actor functions
*
****************************************************************/

AActor::AActor() 
{
	m_pInternalSkeleton = new ASkeleton();
	m_pSkeleton = m_pInternalSkeleton;

	m_BVHController = new BVHController();
	m_BVHController->setActor(this);

	m_IKController = new IKController();
	m_IKController->setActor(this);

	// code to update additional Actor data goes here
	resetGuide();

}

AActor::AActor(const AActor* actor)
{
	*this = *actor;
}

AActor& AActor::operator = (const AActor& actor)
{
	// Performs a deep copy
	if (&actor == this)
	{
		return *this;
	}
	m_pSkeleton = actor.m_pSkeleton;

	// code to update additional Actor data goes here


	return *this;
}

AActor::~AActor()
{
	 delete m_IKController;
	 delete m_BVHController;
	 delete m_pInternalSkeleton;

}

void AActor::clear()
{
	// looks like it is clearing more times than the number of actors.  as a result, m_pSkeleton is not defined for last case.
	m_pSkeleton->clear();  

	// code to update additional Actor data goes here
}

void AActor::update()
{
	if (!m_pSkeleton->getRootNode() )
		 return; // Nothing loaded
	else m_pSkeleton->update();

	// code to update additional Actor data goes here

}

ASkeleton* AActor::getSkeleton()
{
	return m_pSkeleton;
}

void AActor::setSkeleton(ASkeleton* pExternalSkeleton)
{
	m_pSkeleton = pExternalSkeleton;
}

void AActor::resetSkeleton()
{
	m_pSkeleton = m_pInternalSkeleton;
}

BVHController* AActor::getBVHController()
{
	return m_BVHController;
}

IKController* AActor::getIKController()
{
	return m_IKController;
}

mat3 alignVector(vec3 src, vec3 tgt) {
	vec3 axis = src.Cross(tgt);
	float dot_prod = Dot(src, tgt);
	float k = 1.0f / (1.0f + dot_prod);
	mat3 res;
	res[0][0] = axis[0] * axis[0] * k + dot_prod;
	res[0][1] = axis[1] * axis[0] * k - axis[2];
	res[0][2] = axis[2] * axis[0] * k + axis[1];
	res[1][0] = axis[0] * axis[1] * k + axis[2];
	res[1][1] = axis[1] * axis[1] * k + dot_prod;
	res[1][2] = axis[2] * axis[1] * k - axis[0];
	res[2][0] = axis[0] * axis[2] * k - axis[1];
	res[2][2] = axis[2] * axis[2] * k + dot_prod;
	return res;
}
void AActor::updateGuideJoint(vec3 guideTargetPos)
{
	if (!m_pSkeleton->getRootNode()) { 
		return; 
	}
	// TODO: 
	// 1.	Set the global position of the guide joint to the global position of the root joint
	// 2.	Set the y component of the guide position to 0
	// 3.	Set the global rotation of the guide joint towards the guideTarget
	
	vec3 pos = m_Guide.getGlobalRotation() * m_pSkeleton->getRootNode()->getGlobalTranslation() + m_Guide.getGlobalTranslation();
	pos[1] = 0;
	m_Guide.setGlobalTranslation(pos);
	
	// find rotation to align vector
	vec3 forward = (guideTargetPos - m_Guide.getGlobalTranslation()).Normalize();
	vec3 up = vec3(0.0, 1.0, 0.0);
	vec3 look = up.Cross(forward);
	mat3 rot;
	rot[0][0] = look[0];
	rot[1][0] = look[1];
	rot[2][0] = look[2];
	rot[0][1] = up[0];
	rot[1][1] = up[1];
	rot[2][1] = up[2];
	rot[0][2] = forward[0];
	rot[1][2] = forward[1];
	rot[2][2] = forward[2];
	m_Guide.setGlobalRotation(rot);
	
	//m_Guide.updateTransform();
	return;
}

void AActor::solveFootIK(float leftHeight, float rightHeight, bool rotateLeft, bool rotateRight, vec3 leftNormal, vec3 rightNormal)
{
	if (!m_pSkeleton->getRootNode()) { return; }
	AJoint* leftFoot = m_pSkeleton->getJointByID(m_IKController->mLfootID);
	AJoint* rightFoot = m_pSkeleton->getJointByID(m_IKController->mRfootID);

	// TODO: 
	// The normal and the height given are in the world space

	// 1.	Update the local translation of the root based on the left height and the right height

	m_pSkeleton->update();

	// 2.	Update the character with Limb-based IK 
	
	// Rotate Foot
	if (rotateLeft)
	{
		// Update the local orientation of the left foot based on the left normal
		;
	}
	if (rotateRight)
	{
		// Update the local orientation of the right foot based on the right normal
		;
	}
	m_pSkeleton->update();
}
