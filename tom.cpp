// The sample model.  You should build a file
// very similar to this for when you make your model.
#include "modelerview.h"
#include "modelerapp.h"
#include "modelerdraw.h"
#include "bitmap.h"
#include <FL/gl.h>

#include "modelerglobals.h"
#include "HTreeNode.h"
#include "kinematic.h"

#include "particleSystem.h"

// fxck the framework
vector<HTreeNode*> leftHandNodes;
vector<HTreeNode*> rightHandNodes;
vector<HTreeNode*> tailNodes;
float getParticleCount() { return VAL(PARTICLE_COUNT); }

// This is a list of the controls for the RobotArm
// We'll use these constants to access the values 
// of the controls from the user interface.

void drawCones(double x, double y, double h);
void drawTorus(double r, double thickness);
void load_texture();

// To make a TomModel, we inherit off of ModelerView
class TomModel : public ModelerView 
{
	static void drawChest();
	static void drawNeck();
	static void drawBody();
	static void drawUpTail();
	static void drawDownTail();
	static void drawTailEnd();
	static void drawUpArm();
	static void drawDownArm();
	static void drawHand();
	static void drawUpLeg();
	static void drawDownLeg();
	static void drawFoot();
	static void drawHead();
	static void drawLeftEar();
	static void drawRightEar();
	static void drawLeftEye();
	static void drawRightEye();


	HTreeNode* root;
	std::list<HTreeNode*> dynamicNodes;

	vector<HTreeNode*> ikNodes;
	Vec3d ikTarget;
	IKinematic ik;

	void updateDynamics();

public:
    TomModel(int x, int y, int w, int h, char *label) 
        : ModelerView(x,y,w,h,label), root(nullptr) 
	{
		root = new HTreeNode("chest",  &drawChest);
		dynamicNodes.push_back(root);
		Vec3d T, R, S;
		double angle;

		T = { 0.0, 0.0, -1.0 };
		R = { 1.0, 0.0, 0.0 };
		S = { 1.0, 1.0, 1.0 };
		angle = -90;
		HTreeNode* pNeck = new HTreeNode("neck", &drawNeck);
		root->addChild(pNeck, S, T, R, angle);
		dynamicNodes.push_back(pNeck);

		T = { 0.0, 0.0, 1.8 };
		R = { 0.0, 0.0, 0.0 };
		S = { 1.0, 1.0, 1.0 };
		angle = 0;
		HTreeNode* pHead = new HTreeNode("head", &drawHead);
		pNeck->addChild(pHead, S, T, R, angle);
		dynamicNodes.push_back(pHead);

		T = { -1.5, 0.0, 0.4 };
		R = { 1.0, -0.4, 0.4 };
		S = { 1.0, 1.0, 1.0 };
		angle = 90;
		HTreeNode* pLeftEar = new HTreeNode("leftear", &drawLeftEar);
		pHead->addChild(pLeftEar, S, T, R, angle);
		dynamicNodes.push_back(pLeftEar);

		T = { 0.3, 0.0, 1.3 };
		R = { 1.0, 0.4, -0.4 };
		S = { 1.0, 1.0, 1.0 };
		angle = 90;
		HTreeNode* pRightEar = new HTreeNode("rightear", &drawRightEar);
		pHead->addChild(pRightEar, S, T, R, angle);
		dynamicNodes.push_back(pRightEar);

		T = { -0.4, -1.1, 0.2 };
		R = { 0.0, 0.0, 0.0 };
		S = { 1.0, 1.0, 1.3 };
		angle = 0;
		HTreeNode* pLeftEye = new HTreeNode("lefteye", &drawLeftEye);
		pHead->addChild(pLeftEye, S, T, R, angle);
		dynamicNodes.push_back(pLeftEye);

		T = { 0.4, -1.1, 0.2 };
		R = { 0.0, 0.0, 0.0 };
		S = { 1.0, 1.0, 1.3 };
		angle = 0;
		HTreeNode* pRightEye = new HTreeNode("righteye", &drawRightEye);
		pHead->addChild(pRightEye, S, T, R, angle);
		dynamicNodes.push_back(pRightEye);

		// body
		T = { 0.0, -2.99, 0.0 };
		R = { 1.0, 0.0, 0.0 };
		S = { 1.0, 1.0, 1.0 };
		angle = 0;
		HTreeNode* pBody = new HTreeNode("Body", &drawBody);
		root->addChild(pBody, S, T, R, angle);
		//dynamicNodes.push_back(pBody);

		T = { .0, -2.5, -1.95 };
		R = { 1.0, 0.0, 0.0 };
		S = { 1.0, 1.0, 1.0 };
		angle = -170;
		HTreeNode* pUTail = new HTreeNode("UTail", &drawUpTail);
		pBody->addChild(pUTail, S, T, R, angle);
		//dynamicNodes.push_back(pUTail);

		T = { .0, -0.0, 4.0 };
		R = { 0.0, 1.0, 0.0 };
		S = { 1.0, 1.0, 1.0 };
		angle = 20;
		HTreeNode* pDTail = new HTreeNode("DTail", &drawDownTail);
		pUTail->addChild(pDTail, S, T, R, angle);
		//dynamicNodes.push_back(pDTail);

		T = { .0, -0.0, 1.5 };
		R = { 1.0, 0.0, 1.0 };
		S = { 1.0, 1.0, 1.0 };
		angle = 0;
		HTreeNode* pTailEnd = new HTreeNode("TailEnd", &drawTailEnd);
		pDTail->addChild(pTailEnd, S, T, R, angle);
		//dynamicNodes.push_back(pTailEnd);

		T = { 1.25, -0.5, -1.0 };
		R = { 0.0, 0.35, -0.12 };
		S = { 1.0, 1.0, 1.0 };
		angle = 90;
		HTreeNode* pLUArm = new HTreeNode("LUArm", &drawUpArm);
		root->addChild(pLUArm, S, T, R, angle);
		dynamicNodes.push_back(pLUArm);

		T = { -1.25, -0.5, -1.0 };
		R = { 0.0, 0.35, -0.12 };
		S = { 1.0, 1.0, 1.0 };
		angle = -90;
		HTreeNode* pRUArm = new HTreeNode("RUArm", &drawUpArm);
		root->addChild(pRUArm, S, T, R, angle);
		dynamicNodes.push_back(pRUArm);

		T = { 1.0, -2.85, -1.5 };
		R = { 1.0, 0.0, 0.0 };
		S = { 1.0, 1.0, 1.0 };
		angle = 45;
		HTreeNode* pLULeg = new HTreeNode("LULeg", &drawUpLeg);
		pBody->addChild(pLULeg, S, T, R, angle);
		dynamicNodes.push_back(pLULeg);

		T = { -1.0, -2.85, -1.5 };
		R = { 1.0, 0.0, 0.0 };
		S = { 1.0, 1.0, 1.0 };
		angle = 45;
		HTreeNode* pRULeg = new HTreeNode("RULeg", &drawUpLeg);
		pBody->addChild(pRULeg, S, T, R, angle);
		dynamicNodes.push_back(pRULeg);

		T = { 0.0, -0.0, 3.0 };
		R = { 1.0, 1.0, 0.0 };
		S = { 1.0, 1.0, 1.0 };
		angle = -90;
		HTreeNode* pLDArm = new HTreeNode("LDArm", &drawDownArm);
		pLUArm->addChild(pLDArm, S, T, R, angle);
		dynamicNodes.push_back(pLDArm);

		T = { 0.0, -0.0, 3.0 };
		R = { 1.0, 1.0, 0.0 };
		S = { 1.0, 1.0, 1.0 };
		angle = 90;
		HTreeNode* pRDArm = new HTreeNode("RDArm", &drawDownArm);
		pRUArm->addChild(pRDArm, S, T, R, angle);
		dynamicNodes.push_back(pRDArm);

		T = { 0.0, -0.0, 2.5 };
		R = { 1.0, 0.0, 0.0 };
		S = { 1.0, 1.0, 1.0 };
		angle = 25;
		HTreeNode* pLDLeg = new HTreeNode("LDLeg", &drawDownLeg);
		pLULeg->addChild(pLDLeg, S, T, R, angle);
		dynamicNodes.push_back(pLDLeg);

		T = { 0.0, -0.0, 2.5 };
		R = { 1.0, 0.0, 0.0 };
		S = { 1.0, 1.0, 1.0 };
		angle = 25;
		HTreeNode* pRDLeg = new HTreeNode("RDLeg", &drawDownLeg);
		pRULeg->addChild(pRDLeg, S, T, R, angle);
		dynamicNodes.push_back(pRDLeg);

		T = { 0.0, 0.0, 3.0 };
		R = { 1.0, -1.0, 0.0 };
		S = { 1.0, 1.0, 1.0 };
		angle = 90;
		HTreeNode* pLHand = new HTreeNode("LHand", &drawHand);
		pLDArm->addChild(pLHand, S, T, R, angle);
		dynamicNodes.push_back(pLHand);

		T = { 0.0, 0.0, 2.6 };
		R = { 0.0, 0.0, 1.0 };
		S = { 1.0, 1.0, 1.0 };
		angle = -35;
		HTreeNode* pRHand = new HTreeNode("RHand", &drawHand);
		pRDArm->addChild(pRHand, S, T, R, angle);
		dynamicNodes.push_back(pRHand);

		T = { 0.0, 0.0, 2.0 };
		R = { 1.0, 0.0, 0.0 };
		S = { 1.0, 1.0, 1.0 };
		angle = -10;
		HTreeNode* pLFoot = new HTreeNode("LFoot", &drawFoot);
		pLDLeg->addChild(pLFoot, S, T, R, angle);
		dynamicNodes.push_back(pLFoot);

		T = { 0.0, 0.0, 2.0 };
		R = { 1.0, 0.0, 1.0 };
		S = { 1.0, 1.0, 1.0 };
		angle = -10;
		HTreeNode* pRFoot = new HTreeNode("RFoot", &drawFoot);
		pRDLeg->addChild(pRFoot, S, T, R, angle);

		ikNodes.push_back(root);
		ikNodes.push_back(pBody);
		ikNodes.push_back(pUTail);
		ikNodes.push_back(pDTail);
		ikNodes.push_back(pTailEnd);
		ik = IKinematic(ikNodes);

		tailNodes.push_back(root);
		tailNodes.push_back(pBody);
		tailNodes.push_back(pUTail);
		tailNodes.push_back(pDTail);
		tailNodes.push_back(pTailEnd);

		leftHandNodes.push_back(root);
		leftHandNodes.push_back(pLUArm);
		leftHandNodes.push_back(pLDArm);
		leftHandNodes.push_back(pLHand);

		rightHandNodes.push_back(root);
		rightHandNodes.push_back(pRUArm);
		rightHandNodes.push_back(pRDArm);
		rightHandNodes.push_back(pRHand);
	}

    virtual void draw();
};

// We need to make a creator function, mostly because of
// nasty API stuff that we'd rather stay away from.
ModelerView* createTomModel(int x, int y, int w, int h, char *label)
{ 
    return new TomModel(x,y,w,h,label); 
}

// We are going to override (is that the right word?) the draw()
// method of ModelerView to draw out TomModel
void TomModel::draw()
{
    // This call takes care of a lot of the nasty projection 
    // matrix stuff.  Unless you want to fudge directly with the 
	// projection matrix, don't bother with this ...
    ModelerView::draw();
	GLfloat ChangedPosition[] = { VAL(LIGHT_X), VAL(LIGHT_Y), VAL(LIGHT_Z),1 };
	GLfloat ChangedIntensity[] = { VAL(LIGHT_INTENSITY), VAL(LIGHT_INTENSITY), VAL(LIGHT_INTENSITY), 1 };
	glLightfv(GL_LIGHT0, GL_POSITION, ChangedPosition);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, ChangedIntensity);

	glFogf(GL_FOG_DENSITY, VAL(FOG_INDEX)/30);
	//glLightfv(GL_LIGHT0, GL_SPECULAR, ChangedIntensity);

	updateDynamics();
	ik.solve(ikTarget);
	// arg is depth
	root->draw(0);
	endDraw();
}

void TomModel::updateDynamics()
{
	ikTarget[0] = VAL(IK_X);
	ikTarget[1] = VAL(IK_Y);
	ikTarget[2] = VAL(IK_Z);
	ik.setConstrain("Body", VAL(IK_CSTR_1L), VAL(IK_CSTR_1H));
	ik.setConstrain("UTail", VAL(IK_CSTR_2L), VAL(IK_CSTR_2H));
	ik.setConstrain("DTail", VAL(IK_CSTR_3L), VAL(IK_CSTR_3H));
	if (VAL(MOOD) == 0) {
		for (auto n : dynamicNodes) {
			if (strcmp(n->name, "LUArm") == 0) {
				n->T = { 1.25, -0.5, -1.0 };
				n->R = { 0.0, 0.35, -0.12 };
				n->S = { 1.0, 1.0, 1.0 };
				n->angle = 90;
			}
			if (strcmp(n->name, "RUArm") == 0) {
				n->T = { -1.25, -0.5, -1.0 };
				n->R = { 0.0, 0.35, -0.12 };
				n->S = { 1.0, 1.0, 1.0 };
				n->angle = -90;
			}
			if (strcmp(n->name, "RDArm") == 0) {
				n->T = { 0.0, -0.0, 3.0 };
				n->R = { 1.0, 1.0, 0.0 };
				n->S = { 1.0, 1.0, 1.0 };
				n->angle = 90;
			}
			if (strcmp(n->name, "LDArm") == 0) {
				n->T = { 0.0, -0.0, 3.0 };
				n->R = { 1.0, 1.0, 0.0 };
				n->S = { 1.0, 1.0, 1.0 };
				n->angle = -90;
			}
			if (strcmp(n->name, "LDLeg") == 0) {
				n->T = { 0.0, -0.0, 2.5 };
				n->R = { 1.0, 0.0, 0.0 };
				n->S = { 1.0, 1.0, 1.0 };
				n->angle = 25;
			}
			if (strcmp(n->name, "RDLeg") == 0) {
				n->T = { 0.0, -0.0, 2.5 };
				n->R = { 1.0, 0.0, 0.0 };
				n->S = { 1.0, 1.0, 1.0 };
				n->angle = 25;
			}
		}
	}

	else if ((VAL(MOOD) == 1)) {
		for (auto n : dynamicNodes) {
			if (strcmp(n->name, "LUArm") == 0) {
				n->T = { 1.25, -0.5, -1.0 };
				n->R = { 0.0, 0.35, -0.12 };
				n->S = { 1.0, 1.0, 1.0 };
				n->angle = 90;
			}
			if (strcmp(n->name, "RUArm") == 0) {
				n->T = { -1.25, -0.5, -1 };
				n->S = { 1, 1, 1 };
				n->R = { 0.006, 0.864, -1 };
				n->angle = -90;
			}
			if (strcmp(n->name, "RDArm") == 0) {
				n->T = { 0.0, -0.0, 3.0 };
				n->R = { 1.0, 0.006, -1.0 };
				n->S = { 1.0, 1.0, 1.0 };
				n->angle = 90;
			}
			if (strcmp(n->name, "LDArm") == 0) {
				n->T = { 0.0, -0.0, 3.0 };
				n->R = { 0.876, 0.785, 0.345 };
				n->S = { 1.0, 1.0, 1.0 };
				n->angle = -131;
			}
			if (strcmp(n->name, "LDLeg") == 0) {
				n->T = { 0.0, -0.0, 2.5 };
				n->R = { 1.0, 0.0, 0.0 };
				n->S = { 1.0, 1.0, 1.0 };
				n->angle = 25;
			}
			if (strcmp(n->name, "RDLeg") == 0) {
				n->T = { 0.0, -0.0, 2.5 };
				n->R = { 1.0, 0.0, 0.0 };
				n->S = { 1.0, 1.0, 1.0 };
				n->angle = 25;
			}
		}
	}
	else if ((VAL(MOOD) == 2)) {
		for (auto n : dynamicNodes) {
			if (strcmp(n->name, "LUArm") == 0) {
				n->T = { 1.25, -0.5, -1.0 };
				n->R = { 0.0, 0.35, -0.12 };
				n->S = { 1.0, 1.0, 1.0 };
				n->angle = 90;
			}
			if (strcmp(n->name, "RUArm") == 0) {
				n->T = { -1.25, -0.5, -1 };
				n->S = { 1, 1, 1 };
				n->R = { 0.006, 0.864, -1 };
				n->angle = -90;
			}
			if (strcmp(n->name, "RDArm") == 0) {
				n->T = { 0.0, -0.0, 3.0 };
				n->R = { -1.0, -1, -0.797 };
				n->S = { 1.0, 1.0, 1.0 };
				n->angle = -170;
			}
			if (strcmp(n->name, "LDArm") == 0) {
				n->T = { 0.0, -0.0, 3.0 };
				n->R = { 0.876, 0.785, 0.345 };
				n->S = { 1.0, 1.0, 1.0 };
				n->angle = -131;
			}
			if (strcmp(n->name, "LDLeg") == 0) {
				n->T = { 0.0, -0.0, 2.5 };
				n->R = { 1.0, 0.0, 0.0 };
				n->S = { 1.0, 1.0, 1.0 };
				n->angle = 25;
			}
			if (strcmp(n->name, "RDLeg") == 0) {
				n->T = { 0.0, -0.0, 2.5 };
				n->R = { 1.0, 0.0, 0.0 };
				n->S = { 1.0, 1.0, 1.0 };
				n->angle = 25;
			}
		}
	}
	else if ((VAL(MOOD) == 3)) {
		for (auto n : dynamicNodes) {
			if (strcmp(n->name, "LUArm") == 0) {
				n->T = { 1.25, -0.5, -1.0 };
				n->R = { -0.379, 0.751, 0.266 };
				n->S = { 1.0, 1.0, 1.0 };
				n->angle = 99;
			}
			if (strcmp(n->name, "RUArm") == 0) {
				n->T = { -1.25, -0.5, -1.0 };
				n->R = { 1, 1, 0 };
				n->S = { 1.0, 1.0, 1.0 };
				n->angle = -86;
			}
			if (strcmp(n->name, "RDArm") == 0) {
				n->T = { 0.0, -0.0, 3.0 };
				n->R = { -1.0, -1, -0.797 };
				n->S = { 1.0, 1.0, 1.0 };
				n->angle = 0;
			}
			if (strcmp(n->name, "LDArm") == 0) {
				n->T = { 0.0, -0.0, 3.0 };
				n->R = { 0.876, 0.785, 0.345 };
				n->S = { 1.0, 1.0, 1.0 };
				n->angle = 0;
			}
			if (strcmp(n->name, "LDLeg") == 0) {
				n->T = { 0.0, -0.0, 2.5 };
				n->R = { 0.0, 1.0, 0.0 };
				n->S = { 1.0, 1.0, 1.0 };
				n->angle = 100;
			}
			if (strcmp(n->name, "RDLeg") == 0) {
				n->T = { 0.0, -0.0, 2.5 };
				n->R = { 0.0, 1.0, 0.0 };
				n->S = { 1.0, 1.0, 1.0 };
				n->angle = -100;
			}
		}
	}
}

void TomModel::drawChest()
{
	if (VAL(BODYCHANGE) == 0) {

		setAmbientColor(.1f, .1f, .1f);
		setDiffuseColor(COLOR_DARK_GREY);
		glTranslated(-1.5, -3, -2);
		drawBox(3, 3, 2);


		glTranslated(0.5, 0, 0.01);
		setDiffuseColor(COLOR_LIGHT_GREY);
		drawBox(2, 2.5, 2.0);
	}
	else {
		setAmbientColor(.1f, .1f, .1f);
		setDiffuseColor(COLOR_DARK_GREY);
		glTranslated(0.0, -1.5, -1.0);
		drawSphere(1.5);
		glTranslated(0.0, 0.0, 0.2);
		setDiffuseColor(COLOR_LIGHT_GREY);
		drawSphere(1.4);
	}
}

void TomModel::drawBody()
{	
	if (VAL(BODYCHANGE) == 0) {
		setAmbientColor(.1f, .1f, .1f);
		setDiffuseColor(COLOR_DARK_GREY);
		glTranslated(-1.5, -3, -2);
		drawBox(3, 3, 2);
		glTranslated(0.5, 0.5, 0.01);
		setDiffuseColor(COLOR_LIGHT_GREY);
		drawBox(2, 2.5, 2.0);
	}
	else {
		setAmbientColor(.1f, .1f, .1f);
		setDiffuseColor(COLOR_DARK_GREY);
		glTranslated(0.0, -1.5, -1.0);
		drawSphere(1.5);
		glTranslated(0.0, -0.01, 0.2);
		setDiffuseColor(COLOR_LIGHT_GREY);
		drawSphere(1.4);
	}
}

void TomModel::drawUpTail()
{
	setAmbientColor(.1f,.1f,.1f);
	setDiffuseColor(COLOR_DARK_GREY);
	drawSphere(0.5);
	drawCylinder(4, 0.5, 0.3);
}

void TomModel::drawDownTail()
{
	setAmbientColor(.1f,.1f,.1f);
	setDiffuseColor(COLOR_LIGHT_GREY);
	drawSphere(0.3);
	drawCylinder(3, 0.3, 0.1);
}

void TomModel::drawTailEnd() {}

void TomModel::drawUpArm()
{
	setAmbientColor(.1f,.1f,.1f);
	setDiffuseColor(COLOR_DARK_GREY);
	drawSphere(0.75);
	if (VAL(TEXTURE) == 0) {
		drawCylinder(3, 0.75, 0.5);
	}
	else {
		load_texture();
		drawCylinder(3, 0.75, 0.5);
		glDisable(GL_TEXTURE_2D);
	}
}

void TomModel::drawDownArm()
{
	setAmbientColor(.1f,.1f,.1f);
	setDiffuseColor(COLOR_DARK_GREY);
	drawSphere(0.5);
	if (VAL(TEXTURE) == 0) {
		drawCylinder(2.5, 0.5, 0.25);
	}
	else {
		load_texture();
		drawCylinder(2.5, 0.5, 0.25);
		glDisable(GL_TEXTURE_2D);
	}
}

void TomModel::drawHand()
{
	glScaled(1, 1.5, 1);
	setAmbientColor(.1f,.1f,.1f);
	setDiffuseColor(COLOR_LIGHT_GREY);
	drawCylinder(0.2, 0.5, 0.5);
	glTranslated(-0.6, 0.2, 0.1);
	drawSphere(0.25);
	glTranslated(0.4, 0.25, 0.0);
	drawSphere(0.25);
	glTranslated(0.4, 0, 0.0);
	drawSphere(0.25);
	glTranslated(0.4, -0.25, 0.0);
	drawSphere(0.25);
}

void TomModel::drawUpLeg()
{
	setAmbientColor(.1f,.1f,.1f);
	setDiffuseColor(COLOR_DARK_GREY);
	drawSphere(1.0);
	
	if (VAL(TEXTURE) == 0) {
		drawCylinder(2.5, 1.0, 0.75);
	}
	else {
		load_texture();
		drawCylinder(2.5, 1.0, 0.75);
		glDisable(GL_TEXTURE_2D);
	}
}

void TomModel::drawDownLeg()
{
	setAmbientColor(.1f,.1f,.1f);
	setDiffuseColor(COLOR_DARK_GREY);
	drawSphere(0.75);
	
	if (VAL(TEXTURE) == 0) {
		drawCylinder(2, 0.75, 0.5);
	}
	else {
		load_texture();
		drawCylinder(2, 0.75, 0.5);
		glDisable(GL_TEXTURE_2D);
	}
}

void TomModel::drawFoot()
{
	glScaled(1, 1.5, 1);
	setAmbientColor(.1f,.1f,.1f);
	setDiffuseColor(COLOR_LIGHT_GREY);
	drawCylinder(0.3, 0.75, 0.75);
	glTranslated(-0.6, 0.5, 0.1);
	drawSphere(0.25);
	glTranslated(0.4, 0.25, 0.0);
	drawSphere(0.25);
	glTranslated(0.4, 0, 0.0);
	drawSphere(0.25);
	glTranslated(0.4, -0.25, 0.0);
	drawSphere(0.25);

}

void TomModel::drawNeck()
{
	setAmbientColor(.1f, .1f, .1f);
	setDiffuseColor(COLOR_DARK_GREY);
	glTranslated(0.0, 0.0, -0.2);
	drawCylinder(0.9, 0.5, 0.5);
}

void TomModel::drawHead() {
	setAmbientColor(.1f, .1f, .1f);
	setDiffuseColor(COLOR_DARK_GREY);
	if (VAL(HEADCHANGE) == 0)
		drawSphere(1.6);
	else {
		glPushMatrix();
		glTranslated(-1.4, -1.4, -1.4);
		drawBox(2.8, 2.8, 2.8);
		glPopMatrix();
	}

	setAmbientColor(.4f, .4f, .4f);
	setDiffuseColor(.7f, .7f, .1f);
	glTranslated(0, 0, 2.5);
	drawTorus(1.5, 0.2);
}

void TomModel::drawLeftEar() {
	setAmbientColor(.1f, .1f, .1f);
	setDiffuseColor(COLOR_DARK_GREY);
	//drawTriangle(0, 0, 0, 1.6, 0, 0, 0.8, 1.6, 0);
	drawCones(1.6, 0.6, 1.6);
	setDiffuseColor(.3f, .1f, .1f);
	drawTriangle(0.5, 0, 0.31, 1.1, 0, 0.31, 0.8, 1.1, 0.1);
}

void TomModel::drawRightEar() {
	setAmbientColor(.1f, .1f, .1f);
	setDiffuseColor(COLOR_DARK_GREY);
	//drawTriangle(0, 0, 0, 1.6, 0, 0, 0.8, 1.6, 0);
	drawCones(1.6, 0.6, 1.6);
	setDiffuseColor(.3f, .1f, .1f);
	drawTriangle(0.5, 0, 0.31, 1.1, 0, 0.31, 0.8, 1.1, 0.1);
}

void TomModel::drawLeftEye() {
	if (VAL(MOOD) != 2) {
		setAmbientColor(1, 1, 1);
		setDiffuseColor(.7f, .7f, .1f);
		drawSphere(0.5);
		glPushMatrix();
		glScaled(1.2, 0.5, 1.3);
		glTranslated(-0.03, -0.82, 0.0);
		setDiffuseColor(.1f, .9f, .1f);
		drawSphere(0.2);
		setAmbientColor(.1f, .1f, .1f);
		glTranslated(0.0, -0.1, -0.04);
		setDiffuseColor(0.0, 0.0, 0.0);
		drawSphere(0.15);
		glPopMatrix();
	}
	else {
		setAmbientColor(1, 1, 1);
		setDiffuseColor(.7f, .7f, .1f);
		glTranslated(-0.4, -0.44, 0);
		drawBox(0.5, 1, 0.1);
	}
}

void TomModel::drawRightEye() {
	if ((VAL(MOOD) == 0) || (VAL(MOOD) == 3)) {
		setAmbientColor(1, 1, 1);
		setDiffuseColor(.7f, .7f, .1f);
		drawSphere(0.5);
		glPushMatrix();
		glScaled(1.2, 0.5, 1.3);
		glTranslated(-0.03, -0.82, 0.0);
		setDiffuseColor(.1f, .9f, .1f);
		drawSphere(0.2);
		setAmbientColor(.1f, .1f, .1f);
		glTranslated(0.0, -0.1, -0.04);
		setDiffuseColor(0.0, 0.0, 0.0);
		drawSphere(0.15);
		glPopMatrix();
	}
	else if ((VAL(MOOD) == 1)||(VAL(MOOD) == 2)){
		setAmbientColor(1, 1, 1);
		setDiffuseColor(.7f, .7f, .1f);
		glTranslated(0,-0.44, 0);
		drawBox(0.5, 1, 0.1);
	}
}

void load_texture()
{
	int width;
	int height;
	unsigned char* data = readBMP("texture.bmp", width, height);
	GLuint textureCheckerID = 0;
	glGenTextures(1, &textureCheckerID);
	glBindTexture(GL_TEXTURE_2D, textureCheckerID);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	glEnable(GL_TEXTURE_2D);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
	glBindTexture(GL_TEXTURE_2D, textureCheckerID);
}

void drawCones(double x, double y, double h) {
	drawTriangle(0, 0, y / 2, x, 0, y / 2, x / 2, h, 0);
	drawTriangle(0, 0, -y / 2, x, 0, -y / 2, x / 2, h, 0);
	drawTriangle(0, 0, y / 2, 0, 0, -y / 2, x / 2, h, 0);
	drawTriangle(x, 0, y / 2, x, 0, -y / 2, x / 2, h, 0);
}

void drawTorus(double r, double thickness) {
	int ver = 30;
	int hor = 30;
	double x3, y3, z3, x4, y4, z4;

	for (int i = 0; i < ver; i++) {
		glBegin(GL_QUAD_STRIP);
		for (int j = 0; j <= hor; j++) {
			

			double s = (i + 1) % ver + 0.5;
			double t = j % hor;

			double x1 = (r + thickness * cos(s * 2 * M_PI / ver)) * cos(t * 2 * M_PI / hor);
			double y1 = (r + thickness * cos(s * 2 * M_PI / ver)) * sin(t * 2 * M_PI / hor);
			double z1 = thickness * sin(s * 2 * M_PI / ver);
			

			s = i % ver + 0.5;

			double x2 = (r + thickness * cos(s * 2 * M_PI / ver)) * cos(t * 2 * M_PI / hor);
			double y2 = (r + thickness * cos(s * 2 * M_PI / ver)) * sin(t * 2 * M_PI / hor);
			double z2 = thickness * sin(s * 2 * M_PI / ver);
						
			if ((i == 0) && (j == 1)) {
				double norx = (x1 + x2 + x3 + x4) / 4;
				double nory = (y1 + y2 + y3 + y4) / 4;
				double norz = (z1 + z2 + z3 + z4) / 4;
				glNormal3d(norx, nory, norz);
				glVertex3d(x3, y3, z3);
				glVertex3d(x4, y4, z4);
				glVertex3d(x1, y1, z1);
				glVertex3d(x2, y2, z2);
			}
			else if ((i == 0) && (j == 0)) {

			}
			else {
				double norx = (x1 + x2 + x3 + x4) / 4;
				double nory = (y1 + y2 + y3 + y4) / 4;
				double norz = (z1 + z2 + z3 + z4) / 4;
				glNormal3d(norx, nory, norz);
				glVertex3d(x1, y1, z1);
				glVertex3d(x2, y2, z2);
			}
			x3 = x1; y3 = y1; z3 = z1;
			x4 = x2; y4 = y2; z4 = z2;
		}
		glEnd();
	}
}


int main()
{
	// Initialize the controls
	// Constructor is ModelerControl(name, minimumvalue, maximumvalue, 
	// stepsize, defaultvalue)
    ModelerControl controls[NUMCONTROLS];
    controls[LVL] = ModelerControl("Hireachy level", 0, 5, 1, 5);
	controls[LIGHT_X] = ModelerControl("Light X Position", -30.0, 30.0, 0.01f, 8);
	controls[LIGHT_Y] = ModelerControl("Light Y Position", -30.0, 30.0, 0.01f, 12);
	controls[LIGHT_Z] = ModelerControl("Light Z Position", -30.0, 30.0, 0.01f, 22);
	controls[LIGHT_INTENSITY] = ModelerControl("Light Intensity", 0.0, 5.0, 0.01f, 1.3);
	controls[FOG_INDEX] = ModelerControl("Fog Density", 0.0, 3.0, 1, 0.0);
	controls[BODYCHANGE] = ModelerControl("Change Body Component", 0.0, 1.0, 1, 0.0);
	controls[HEADCHANGE] = ModelerControl("Change Head Component", 0.0, 1.0, 1, 0.0);
	controls[TEXTURE] = ModelerControl("Change Texture", 0.0, 1.0, 1, 0.0);
    controls[IK_X] = ModelerControl("IK X Position", -1.0, 1.0, 0.001f, 0.0);
    controls[IK_Y] = ModelerControl("IK Y Position", -10.0, 10.0, 0.001f, 0.0);
    controls[IK_Z] = ModelerControl("IK Z Position", -10.0, -0.0, 0.001f, -7.0);
    controls[IK_CSTR_1L] = ModelerControl("IK Body Constrain Low", -180, 180, 1, 0);
    controls[IK_CSTR_1H] = ModelerControl("IK Body Constrain High", -180, 180, 1, 90);
    controls[IK_CSTR_2L] = ModelerControl("IK Up Tail Constrain Low", -180, 180, 1, 0);
    controls[IK_CSTR_2H] = ModelerControl("IK Up Tail Constrain High", -180, 180, 1, 0);
    controls[IK_CSTR_3L] = ModelerControl("IK Down Tail Constrain Low", -180, 180, 1, 0);
    controls[IK_CSTR_3H] = ModelerControl("IK Down Tail Constrain High", -180, 180, 1, 0);
	controls[MOOD] = ModelerControl("Change Mood", 0.0, 3.0, 1, 0.0);
	controls[PARTICLE_COUNT] = ModelerControl("Particle count (number per second)", 0.0, 60, 1, 30.0);

    ModelerApplication::Instance()->Init(&createTomModel, controls, NUMCONTROLS);
	ParticleSystem* ps = new ParticleSystem();
	ps->setNodes(0, &leftHandNodes);
	ps->setNodes(1, &rightHandNodes);
	ps->setNodes(2, &tailNodes);
	ps->setParticleCount(0, &getParticleCount);
	ps->setParticleCount(1, &getParticleCount);
	ps->setParticleCount(2, &getParticleCount);
	ModelerApplication::Instance()->SetParticleSystem(ps);
    return ModelerApplication::Instance()->Run();
}


