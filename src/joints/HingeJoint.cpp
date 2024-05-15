#include "HingeJoint.hpp"


namespace mars
{
    namespace ode_physics
    {

        using namespace utils;
        using namespace interfaces;

        HingeJoint::HingeJoint(WorldPhysics *world,
                               data_broker::DataBrokerInterface *dataBroker,
                               configmaps::ConfigMap &config)
            : Joint(world, dataBroker, config)
        {
            Joint::createJoint();
        }

        HingeJoint::~HingeJoint(void)
        {
        }

        Joint* HingeJoint::instantiate(WorldPhysics *world,
                                       data_broker::DataBrokerInterface *dataBroker,
                                       configmaps::ConfigMap &config)
        {
            return new HingeJoint(world, dataBroker, config);
        }

        bool HingeJoint::createJoint(dBodyID body1, dBodyID body2)
        {
            jointId= dJointCreateHinge(theWorld->getWorld(),0);
            dJointAttach(jointId, body1, body2);
            dJointSetHingeAnchor(jointId, config["anchor"]["x"], config["anchor"]["y"],
                                 config["anchor"]["z"]);
            dJointSetHingeAxis(jointId, config["axis1"]["x"], config["axis1"]["y"],
                               config["axis1"]["z"]);

            if(damping>0.00000001)
            {
                dJointSetHingeParam(jointId, dParamFMax, damping);
                dJointSetHingeParam(jointId, dParamVel, 0);
            }
            if(spring > 0.00000001)
            {
                dJointSetHingeParam(jointId, dParamLoStop, lo1);
                dJointSetHingeParam(jointId, dParamHiStop, hi1);
                dJointSetHingeParam(jointId, dParamStopCFM, cfm1);
                dJointSetHingeParam(jointId, dParamStopERP, erp1);
            } else if(lo1 != 0)
            {
                dJointSetHingeParam(jointId, dParamLoStop, lo1);
                dJointSetHingeParam(jointId, dParamHiStop, hi1);
            }
            if(jointCFM > 0)
            {
                dJointSetHingeParam(jointId, dParamCFM, jointCFM);
            }
            // good value for the SpaceClimber robot
            //dJointSetHingeParam(jointId, dParamCFM, 0.03);
            //dJointSetHingeParam(jointId, dParamCFM, 0.018);
            //dJointSetHingeParam(jointId, dParamCFM, 0.09);

            return true;
        }

        void HingeJoint::updateState()
        {
            const dReal *b1_pos, *b2_pos;
            dReal anchor[4], axis[4], axis2[4];
            int calc1 = 0, calc2 = 0;
            dReal radius, dot, torque;
            dReal v1[3], normal[3], load[3], tmp1[3], axis_force[3];
            // todo: - check where we need locking
            //       - handle invert and joint offset mechanism
            //MutexLocker locker(&(theWorld->iMutex));

            position1 = -1*(sReal)dJointGetHingeAngle(jointId);
            dJointGetHingeAnchor(jointId, anchor);
            dJointGetHingeAxis(jointId, axis);
            axis1_torque.setZero();
            axis2_torque.setZero();
            velocity1 = getVelocity();
            this->anchor.x() = anchor[0];
            this->anchor.y() = anchor[1];
            this->anchor.z() = anchor[2];
            getAxis(&axis1);
            getForce1(&f1);
            getTorque1(&t1);
            motor_torque = feedback.lambda*-1;
            if(body1)
            {
                b1_pos = dBodyGetPosition(body1);
                dOP(v1, -, b1_pos, anchor);
                //radius = dLENGTH(v1);
                dCROSS(normal, =, axis, v1);
                dot = dDOT(normal, feedback.f1);
                dOPEC(normal, *=, dot);
                dOP(load, -, feedback.f1, normal);
                dCROSS(tmp1, =, v1, normal);
                axis1_torque.x() = (sReal)tmp1[0];
                axis1_torque.y() = (sReal)tmp1[1];
                axis1_torque.z() = (sReal)tmp1[2];
                dCROSS(tmp1, =, v1, load);
                joint_load.x() = (sReal)tmp1[0];
                joint_load.y() = (sReal)tmp1[1];
                joint_load.z() = (sReal)tmp1[2];
                // now nearly the same for the torque
                dot = dDOT(axis, feedback.t1);
                dOPC(tmp1, *, axis, dot);
                dOP(load, -, feedback.t1, tmp1);
                axis1_torque.x() += (sReal)tmp1[0];
                axis1_torque.y() += (sReal)tmp1[1];
                axis1_torque.z() += (sReal)tmp1[2];
                axis1_torque *= -1;
                joint_load.x() += (sReal)load[0];
                joint_load.y() += (sReal)load[1];
                joint_load.z() += (sReal)load[2];
                joint_load *= -1;
            } else if(body2)
            {
                // now we do it correct
                // first get the position vector v1
                b2_pos = dBodyGetPosition(body2);
                dOP(v1, -, b2_pos, anchor);
                
                // then differentiate the torque from the force
                dot = dDOT(feedback.f2, v1) / dDOT(v1, v1);
                dOPC(axis_force, *, v1, dot);
                
                // the difference is the torque
                dOP(tmp1, -, feedback.f2, axis_force);
                
                // the torque value is given by:
                torque = dLENGTH(tmp1) / dLENGTH(v1);
                // then get the normal to v1 and the torque vector
                dCROSS(normal, =, v1, tmp1);
                // and scale the normal to represent the correct torque length
                dOPEC(normal, *=, torque / dLENGTH(normal));
                // now the normal represents the torque vector at the anchor
                // and we make a projection to the axis
                dot = dDOT(normal, axis);
                dOPC(tmp1, *, axis, dot);
                dOP(load, -, normal, tmp1);
                //dCROSS(tmp1, =, v1, normal);
                axis1_torque.x() = (sReal)tmp1[0];
                axis1_torque.y() = (sReal)tmp1[1];
                axis1_torque.z() = (sReal)tmp1[2];
                //dCROSS(tmp1, =, v1, load);
                joint_load.x() = (sReal)load[0];
                joint_load.y() = (sReal)load[1];
                joint_load.z() = (sReal)load[2];
                // now nearly the same for the torque
                dot = dDOT(feedback.t2, axis);
                dOPC(tmp1, *, axis, dot);
                dOP(load, -, feedback.t2, tmp1);
                joint_load.x() += (sReal)load[0];
                joint_load.y() += (sReal)load[1];
                joint_load.z() += (sReal)load[2];
                // TODO: check that this is correct
                axis1_torque *= -1;
                joint_load *= -1;
            }
        }

        std::vector<std::string> HingeJoint::getEditPattern(const std::string& basePath) const
        {
            std::vector<std::string> result{Joint::getEditPattern(basePath)};
            result.push_back(basePath + "axis/*");
            result.push_back(basePath + "anchor/*");
            return result;
        }
    } // end of namespace ode_physics
} // end of namespace mars
