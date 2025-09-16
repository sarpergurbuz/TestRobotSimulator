//
// Created by Andrei on 19.09.22.
//

#include <AndreiUtils/classes/graph/Graph.hpp>
#include <AndreiUtils/classes/RandomNumberGenerator.hpp>
#include <AndreiUtils/classes/PoseDecoupledInterpolator.hpp>
#include <AndreiUtils/classes/Timer.hpp>
#include <AndreiUtils/utilsEigenGeometry.h>
#include <AndreiUtils/utilsEigenGeometry.hpp>
#include <AndreiUtils/utilsGeometry.h>
#include <AndreiUtils/utilsJson.h>
#include <AndreiUtils/utilsJsonEigen.hpp>
#include <AndreiUtils/utilsPose.hpp>
#include <AndreiUtils/utilsThread.h>
#include <ConceptLibrary/abilities/MoveRobotBodyCartesianAbility.h>
#include <ConceptLibrary/concepts/ConceptManager.h>
#include <ConceptLibrary/abilities/MoveGripperAbility.h>
#include <ConceptLibrary/abilities/SetObjectInGripperAbility.h>
#include <ConceptLibrary/abilities/MoveRobotBodyCartesianWithIntermediateGoalsAbility.h>
#include <ConceptLibrary/abilities/ClearObjectInGripperAbility.h>
#include <ConceptLibrary/abilities/LocalizeObjectAbility.h>
#include <ConceptLibrary/abilities/IdleAgentAbility.h>
#include <ConceptLibrary/abilities/SeeThenMoveToObjectAbility.h>
#include <ConceptLibrary/entities/FrankaRobotConcept.h>
#include <ConceptLibrary/entities/GraspableObjectConcept.h>
#include <ConceptLibrary/entities/OpenableObjectConcept.h>
#include <ConceptLibrary/entities/PourerConcept.h>
#include <ConceptLibrary/entities/PourerSurfaceConcept.h>
#include <ConceptLibrary/entities/PouringSurfaceConcept.h>
#include <ConceptLibrary/functions/core/EnvironmentDataFunctions.h>
#include <ConceptLibrary/functions/ExecutorFunctions.h>
#include <ConceptLibrary/functions/BehaviorTreeFunctions.h>
#include <ConceptLibrary/functions/EnvironmentDataFunctions.h>
#include <ConceptLibrary/functions/InstanceFunctions.h>
#include <ConceptLibrary/instances/utils.h>
#include <ConceptLibrary/LibraryInit.h>
#include <ConceptLibrary/skills/GraspSkill.h>
#include <ConceptLibrary/skills/TransportSkill.h>
#include <ConceptLibrary/skills/ReleaseSkill.h>
#include <ConceptLibrary/skills/PourSkill.h>
#include <ConceptLibrary/utils.h>
#include <ConceptLibrary/valueDomains/variations/utilsJson.hpp>
#include <iostream>
#include <PerceptionData/detections/ObjectTagModelDetection.h>
#include <RobotModelling/PathController.h>
#include <RobotModelling/Robot.h>
#include <RobotModelling/SimulationInterface.h>
#include <RobotModelling/SimulationObjectRelativeGrasp.h>
#include <RobotModelling/utils.h>
#include <ConceptLibrary/valueDomains/Pose.h>
#include <limits>
#include <unordered_set>

using namespace AndreiUtils;
using namespace ConceptLibrary;
using namespace DQ_robotics;
using namespace Eigen;
using namespace std;
using namespace PerceptionData;
using namespace RobotModelling;


using ObjectInstance = Instance<ConceptList<ObjectConcept>>;
using SurfaceInstance = Instance<ConceptList<SurfaceConcept>>;
using Vector7d = Eigen::Matrix<double, 7, 1>;

Posed samplePose(double positionVolume = 1, bool keepNonNegative = false, bool orientationOnlyZ = true) {
    Quaterniond q = orientationOnlyZ ? qzRotation(double01Sampler.sample()) : sampleOrientation();
    Vector3d t = keepNonNegative ? Vector3d(double01Sampler.sample(), double01Sampler.sample(),
                                            double01Sampler.sample()) : sampleDirection();
    t *= positionVolume;
    return {q, t};
}

void setSimulationPose(SimulationInterface &sim, string const &objectName, Posed const &pose,
                       map<string, string> const &nameConvertor,
                       map<string, Eigen::Vector3d> const &objectSpecificTranslationAdjustment) {
    auto t = pose.getTranslation();  // add model-specific offset
    Eigen::Vector3d objectSpecificOffset;
    if (AndreiUtils::mapGetIfContains(objectSpecificTranslationAdjustment, objectName, objectSpecificOffset)) {
        t += objectSpecificOffset;
    }
    std::string simulationName;
    if (!AndreiUtils::mapGetIfContains(nameConvertor, objectName, simulationName)) {
        simulationName = objectName;
    }
    sim.get().set_object_pose(simulationName, fromPoseToDQ({pose.getRotation(), t}));
}

void setSimulationPose(SimulationInterface &sim, ObjectTagModel const &object, map<string, string> const &nameConvertor,
                       map<string, Eigen::Vector3d> const &objectSpecificTranslationAdjustment) {
    setSimulationPose(sim, object.name, object.getPose(), nameConvertor, objectSpecificTranslationAdjustment);
}

void setSimulationPose(SimulationInterface &sim, ObjectInstance const &object, map<string, string> const &nameConvertor,
                       map<string, Eigen::Vector3d> const &objectSpecificTranslationAdjustment) {
    auto const objectPoseVal = GetInstancePose::eval(object);
    setSimulationPose(sim, object.instanceId.s, objectPoseVal->q, nameConvertor, objectSpecificTranslationAdjustment);
}

void moveTo(SimulationInterface &sim, std::string const &objectName, Posed const &fromPose, Posed const &toPose,
            map<string, string> const &nameConvertor,
            map<string, Eigen::Vector3d> const &objectSpecificTranslationAdjustment, int nrSteps = 1000) {
    // PoseInterpolator<double> pi;
    // PoseDecoupledInterpolator<double> pdi;
    // pi.compute(milkPose, goalPose, 1000, false, true);  // DualQuaternion powScrew does not work when orientations are the same!

    auto res = sclerp(fromPoseToDQ(fromPose), nrSteps, fromPoseToDQ(toPose));
    for (int i = 0; i < res.cols(); i++) {
        DQ nextPose(res.col(i));
        setSimulationPose(sim, objectName, fromDQToPose(nextPose), nameConvertor, objectSpecificTranslationAdjustment);
        AndreiUtils::sleepMSec(10);
    }
}

void moveObjectToGoal(AndreiUtils::Posed const &goalPose, ObjectInstance const &milk, PathController &path,
                      RobotModelling::Robot &robot, DQ const &endEffectorToObjectOriginDQ, int waitTimeMSec = 0,
                      bool verbose = false) {
    if (verbose) {
        cout << "goal pose = " << goalPose.toString() << endl;
    }
    path.simulationControlToDestination(&robot, fromPoseToDQ(goalPose) * endEffectorToObjectOriginDQ);
    SetInstancePose::eval(milk, ConceptLibrary::Pose{goalPose});
    if (waitTimeMSec > 0) {
        AndreiUtils::sleepMSec(waitTimeMSec);
    }
}

Posed computePourerOrientationGoalPose(
        AndreiUtils::Posed const &pourerPose, ObjectSurface const &pourerSurface, ObjectSurface const &pouringSurface,
        Vector3d const &direction, Vector3d &pourRotationAxis, bool verbose = false) {
    Vector3d const &pouringNormal = pouringSurface.getSurfaceNormal();
    Vector3d const &pourerNormal = pourerSurface.getSurfaceNormal();
    pourRotationAxis = zAxis3d<double>().cross(direction).normalized();  // in pouring surface coordinate frame
    if (verbose) {
        cout << "Pour rotation axis: " << pourRotationAxis.transpose() << endl;
    }
    Vector3d pourerNormalInPouringCF = qRotate(
            qFromRotationMatrix(pouringSurface.getSurfaceCoordinateFrame()).inverse(), pourerNormal);
    pourerNormalInPouringCF.z() = 0;  // project to surface
    pourerNormalInPouringCF.normalize();  // make unit vector
    double pourerOrientationRotationAngle = atan2(pourRotationAxis.dot(pourerNormalInPouringCF),
                                                  direction.dot(pourerNormalInPouringCF));
    if (verbose) {
        cout << "Pourer orientation angle = " << pourerOrientationRotationAngle << endl;
    }
    Quaterniond milkOrientationRotation(AngleAxisd(-pourerOrientationRotationAngle, pouringNormal));
    if (verbose) {
        cout << "Pourer position: " << pourerPose.translationToString() << endl;
    }
    return {milkOrientationRotation * pourerPose.getRotation(), pourerPose.getTranslation()};
}

Posed computePouringAngleGoalPose(double const &pouringAngle, Posed const &pourerPose, Vector3d const &pourRotationAxis,
                                  ObjectSurface const &pourerSurface, ObjectSurface const &pouringSurface,
                                  bool verbose = false) {
    Vector3d const &pouringNormal = pouringSurface.getSurfaceNormal();
    Vector3d const &pourerNormal = pourerSurface.getSurfaceNormal();
    double pourRotationAngle = pouringAngle - vectorAngleAlreadyNormalized(
            pouringNormal, pourerNormal);  // both vectors are in the world coordinate frame
    Vector3d pourRotationAxisInWorldCF = qRotate(qFromRotationMatrix(pouringSurface.getSurfaceCoordinateFrame()),
                                                 pourRotationAxis);
    Quaterniond milkPourRotation(AngleAxisd(-pourRotationAngle, pourRotationAxisInWorldCF));
    if (verbose) {
        cout << "Check: " << pourRotationAxisInWorldCF.dot(pourerNormal) << " == 0?" << endl;
    }
    return {milkPourRotation * pourerPose.getRotation(), pourerPose.getTranslation()};
}

void initializeSimData(map<string, string> &nameConvertor,
                       map<string, Eigen::Vector3d> &objectSpecificTranslationAdjustment) {
    nameConvertor = {
            {"MilkCartonLidlInstance1",   "/MilkCartonLidlInstance1"},
            //{"trash_can",                 "/TrashCan"},
            {"CerealBoxVitalisInstance1", "/CerealBox"},
            {"BowlGreyIkeaInstance",      "/BowlGreyIkeaInstance"},
            {"BowlWhiteIkeaInstance",     "/BowlWhiteIkeaInstance"},
            {"PlasticCupInstance1",       "/PlasticCupInstance1"},
            {"PlasticCupInstance2",       "/PlasticCupInstance2"},
    };
    // {"MilkCartonLidlInstance1",   Eigen::Vector3d{0.03898549, 0.03877477, 0.0965788498305}
    objectSpecificTranslationAdjustment = {
            {"MilkCartonLidlInstance1",   Eigen::Vector3d{0, 0, 0}},
            {"trash_can",                 Eigen::Vector3d{0, 0, 0.2}},
            {"CerealBoxVitalisInstance1", Eigen::Vector3d{0, 0, 0.08}},
            {"BowlGreyIkeaInstance",      Eigen::Vector3d{0, 0, 0}},
            {"BowlWhiteIkeaInstance",     Eigen::Vector3d{0, 0, 0}},
            {"PlasticCupInstance1",       Eigen::Vector3d{0, 0, 0.0}},
            {"PlasticCupInstance2",       Eigen::Vector3d{0, 0, 0.0}},
    };
}

void generateMotion() {
    SimulationInterface &sim = SimulationInterface::getInterface();
    ObjectInstance bowl("BowlGreyIkeaInstance"), milk("MilkCartonLidlInstance1");
    SurfaceInstance bowlSurfaceInstance, milkSurfaceInstance;
    ObjectSurface *bowlSurface, *milkSurface;
    map<string, string> nameConvertor;
    map<string, Eigen::Vector3d> objectSpecificTranslationAdjustment;
    initializeSimData(nameConvertor, objectSpecificTranslationAdjustment);
    String index;
    bool foundSurface = false;
    for (auto &s: bowl.parameters->getValue<ObjectConcept::surfacesProperty>()) {
        if (isASubConceptOfB(s.second.instanceId.s, "PouringSurface")) {
            index = s.first;
            foundSurface = true;
            break;
        }
    }
    assert (foundSurface);
    bowlSurfaceInstance = mapGet(bowl.parameters->getValue<ObjectConcept::surfacesProperty>().m, index);
    cout << "BowlSurfaceInstance " << bowlSurfaceInstance.instanceId.s << ": " << bowlSurfaceInstance.geometry.get()
         << endl;
    bowlSurface = dynamic_pointer_cast<ObjectSurface>(bowlSurfaceInstance.geometry).get();
    auto const bowlSurfaceInstancePoseVal = GetInstancePose::eval(bowlSurfaceInstance);
    cout << "Bowl Surface Instance pose: " << bowlSurfaceInstancePoseVal->q.toString() << endl;
    cout << "Bowl Surface pose: " << bowlSurface->getPose().toString() << endl;
    foundSurface = false;
    for (auto &s: milk.parameters->getValue<ObjectConcept::surfacesProperty>()) {
        if (isASubConceptOfB(s.second.instanceId.s, "PouringSurface")) {
            index = s.first;
            foundSurface = true;
        }
    }
    assert (foundSurface);
    milkSurfaceInstance = mapGet(milk.parameters->getValue<ObjectConcept::surfacesProperty>().m, index);
    cout << "MilkSurfaceInstance " << milkSurfaceInstance.instanceId.s << ": " << milkSurfaceInstance.geometry.get()
         << endl;
    milkSurface = dynamic_pointer_cast<ObjectSurface>(milkSurfaceInstance.geometry).get();
    auto const milkSurfaceInstancePoseVal = GetInstancePose::eval(milkSurfaceInstance);
    cout << "Milk Surface Instance pose: " << milkSurfaceInstancePoseVal->q.toString() << endl;
    cout << "Milk Surface pose: " << milkSurface->getPose().toString() << endl;

    ConceptParameters pourParameters;
    pourParameters.setPropertyValue("angle", Number(double01Sampler.sample() * M_PI_2 + M_PI_2));
    pourParameters.setPropertyValue("height", Number(double01Sampler.sample() * 0.3 + 0.02));
    pourParameters.setPropertyValue("direction", Number(double01Sampler.sample() * 2 * M_PI - M_PI));
    pourParameters.setPropertyValue("time", Number(double01Sampler.sample() * 2 + 1));
    pourParameters.setPropertyValue("from", milk);
    pourParameters.setPropertyValue("into", bowl);
    pourParameters.setPropertyValue("pourerSurface", milkSurfaceInstance);
    pourParameters.setPropertyValue("pouringSurface", bowlSurfaceInstance);

    // TODO: this solution does not work when the pourer is not z-axis aligned//oriented

    SetInstancePose::eval(milk, ConceptLibrary::Pose{samplePose(1, true, true)});
    // milk.resetOrigin({qyRotation(M_PI_2), Vector3d(0.5, 0.5, 1)});
    SetInstancePose::eval(bowl, ConceptLibrary::Pose{AndreiUtils::Posed{qzRotation(double01Sampler.sample()), Vector3d::Zero()}});

    setSimulationPose(sim, bowl, nameConvertor, objectSpecificTranslationAdjustment);
    setSimulationPose(sim, milk, nameConvertor, objectSpecificTranslationAdjustment);
    AndreiUtils::sleepMSec(500);

    double const &pourAngle = pourParameters.getValue<PourSkill::angleProperty>().n;
    double const &pourDirection = pourParameters.getValue<PourSkill::directionProperty>().n;
    double const &pourHeight = pourParameters.getValue<PourSkill::heightProperty>().n;
    double const &pourTime = pourParameters.getValue<PourSkill::timeProperty>().n;
    cout << "Pour direction = " << pourDirection << endl;
    Vector3d direction = Vector3d(cos(pourDirection), sin(pourDirection), 0);
    double l = 0.5;
    Vector3d milkGoalPosition =
            bowlSurface->getSurfaceCenter() + bowlSurface->getSurfaceCoordinateFrame() * direction * l;
    cout << direction.transpose() << endl;

    cout << "Move the object to the interaction volume boundary along the pouring direction" << endl;
    auto milkPoseVal = GetInstancePose::eval(milk);
    Posed goalPose = {milkPoseVal->q.getRotation(), milkGoalPosition};
    moveTo(sim, milk.instanceId.s, milkPoseVal->q, goalPose, nameConvertor, objectSpecificTranslationAdjustment, 250);
    SetInstancePose::eval(milk, ConceptLibrary::Pose{goalPose});

    AndreiUtils::sleepMSec(500);

    cout << "Orient the object correctly towards the pouring axis" << endl;
    Vector3d pourRotationAxis;
    milkPoseVal = GetInstancePose::eval(milk);
    goalPose = computePourerOrientationGoalPose(milkPoseVal->q, *milkSurface, *bowlSurface, direction,
                                                pourRotationAxis);
    cout << goalPose.translationToString() << endl;
    moveTo(sim, milk.instanceId.s, milkPoseVal->q, goalPose, nameConvertor, objectSpecificTranslationAdjustment, 250);
    SetInstancePose::eval(milk, ConceptLibrary::Pose{goalPose});

    AndreiUtils::sleepMSec(500);

    cout << "Going to the correct height" << endl;
    goalPose = computePouringAngleGoalPose(pourAngle, milkPoseVal->q, pourRotationAxis, *milkSurface, *bowlSurface);
    auto tmpMilk = dynamic_pointer_cast<ObjectTagModel>(milk.geometry)->transform(goalPose, false);
    Vector3d positionOverSurfaceCenter = pourHeight * bowlSurface->getSurfaceNormal() + bowlSurface->getSurfaceCenter();
    ObjectSurface tmpMilkSurface = *dynamic_pointer_cast<ObjectSurface>(
            mapGet(tmpMilk->getObjectSurfaces(), milkSurface->name));
    Vector3d d = positionOverSurfaceCenter - tmpMilkSurface.getSurfaceCenter();
    milkPoseVal = GetInstancePose::eval(milk);
    Posed tmpGoalPose = {milkPoseVal->q.getRotation(), milkPoseVal->q.getTranslation() + d};
    moveTo(sim, milk.instanceId.s, milkPoseVal->q, tmpGoalPose, nameConvertor, objectSpecificTranslationAdjustment, 250);

    AndreiUtils::sleepMSec(500);

    cout << "Pouring..." << endl;
    goalPose = goalPose.addTranslation(d);
    moveTo(sim, milk.instanceId.s, tmpGoalPose, goalPose, nameConvertor, objectSpecificTranslationAdjustment, 250);

    AndreiUtils::sleepMSec(int(1000 * pourTime));  // use pour time to sleep

    moveTo(sim, milk.instanceId.s, goalPose, milkPoseVal->q, nameConvertor, objectSpecificTranslationAdjustment, 250);

    cout << "End!" << endl;
    AndreiUtils::sleepMSec(500);
}

bool graspObject(ObjectInstance &obj, std::map<std::string, std::string> const &nameConvertor, SimulationInterface &sim,
                 PathController &path, RobotModelling::Robot &robot, AndreiUtils::Posed &endEffectorToObjectOrigin,
                 nlohmann::json const &config, int waitTimeMSec) {
    auto grasps = createObjectInstanceGeometryFor(obj.instanceId).grasps;
    if (grasps.empty()) {
        throw runtime_error("Can not grasp object with no grasp points!");
    }
    if (!config.contains("graspIndex")) {
        throw std::runtime_error("Config must specify which of the grasps to use for " + obj.instanceId.s);
    }
    int graspIndex = AndreiUtils::mapGet(config.at("graspIndex").get<std::map<std::string, int>>(), obj.instanceId.s);
    if (graspIndex < 0 || graspIndex > grasps.size()) {
        throw std::runtime_error("Grasp index (" + std::to_string(graspIndex) + ") is invalid for vector of size " +
                                 std::to_string(grasps.size()));
    }
    Grasp const &grasp = grasps[graspIndex];

    auto const &simObjectName = AndreiUtils::mapGet(nameConvertor, obj.instanceId.s);
    SimulationObjectRelativeGrasp simGrasp(sim, simObjectName, grasp.getGraspPointPoses(), false,
                                           grasp.getGraspAngleRange());
    Eigen::VectorXd jointValues;
    AndreiUtils::Pose W_q_EE;
    Gripper *const &gripper = robot.getGripper();
    bool computeRandomGrasp = config.contains("useRandomGrasp") && config.at("useRandomGrasp").get<bool>();
    gripper->compute(&simGrasp, &jointValues, nullptr, &W_q_EE, nullptr, computeRandomGrasp);

    // two-step grasping: 1) go behind object at approach direction
    AndreiUtils::Posed endEffectorApproachToWorldPose = W_q_EE * AndreiUtils::Posed(
            qIdentity<double>(), -obj.parameters->getValue<ObjectConcept::interactionVolumeProperty>().n * 0.7 *
                                 zAxis3d<double>());
    path.simulationControlToDestination(&robot, fromPoseToDQ(endEffectorApproachToWorldPose));
    AndreiUtils::sleepMSec(waitTimeMSec);

    // two-step grasping: 2) go to grasp pose
    path.simulationControlToDestination(&robot, fromPoseToDQ(W_q_EE));
    AndreiUtils::sleepMSec(waitTimeMSec);

    /*
    Posed gripperToAttachPointPose = fromDQToPose(
            sim.get().get_object_pose(gripper->getAttachPointName(), -1, DQ_VrepInterface::OP_BLOCKING).inv() *
            sim.get().get_object_pose(gripper->getGripperName(), -1, DQ_VrepInterface::OP_BLOCKING));
    displayData.setPose(gripper->getGripperName(), attachPointToWorldPose * gripperToAttachPointPose);
    //*/

    // grasp object
    gripper->setJointValues(jointValues);
    // attach to object
    sim.get().set_object_parent(simObjectName, gripper->getGripperName(), true);
    AndreiUtils::sleepMSec(waitTimeMSec);

    auto const objPoseVal = GetInstancePose::eval(obj);
    endEffectorToObjectOrigin = getRelativePoseCommonEnd(W_q_EE, objPoseVal->q);
    return true;
}

bool releaseObject(ObjectInstance &obj, AndreiUtils::Posed const &releaseGoalPose,
                   std::map<std::string, std::string> const &nameConvertor, SimulationInterface &sim,
                   PathController &path, RobotModelling::Robot &robot, DQ const &endEffectorToObjectOriginDQ,
                   int waitTimeMSec) {
    // two-step release: 1) go to release pose
    moveObjectToGoal(releaseGoalPose, obj, path, robot, endEffectorToObjectOriginDQ, waitTimeMSec);

    // grasp object
    Gripper *const &gripper = robot.getGripper();
    gripper->setJointValues(Eigen::Matrix<double, 1, 1>(0.0));
    // detach from object
    removeObjectFromGripper(sim.get(), AndreiUtils::mapGet(nameConvertor, obj.instanceId.s));
    AndreiUtils::sleepMSec(waitTimeMSec);

    // two-step release: 2) go back to approach direction
    auto endEffectorToWorldPose = fromDQToPose(robot.getCurrentRobotEEPose());
    AndreiUtils::Posed endEffectorApproachToWorldPose = endEffectorToWorldPose * AndreiUtils::Posed(
            qIdentity<double>(), -obj.parameters->getValue<ObjectConcept::interactionVolumeProperty>().n * 0.7 *
                                 zAxis3d<double>());
    path.simulationControlToDestination(&robot, fromPoseToDQ(endEffectorApproachToWorldPose));
    AndreiUtils::sleepMSec(waitTimeMSec);

    return true;
}

void pourFromInto(ObjectInstance const &from, ObjectInstance const &into, ConceptParameters const &pourParameters,
                  SimulationInterface &sim, PathController &path, Robot &robot, DQ const &endEffectorToObjectOriginDQ,
                  double const &amountToTimeScalingFactor = 32) {
    String index;
    bool foundSurface = false;
    for (auto &s: into.parameters->getValue<ObjectConcept::surfacesProperty>()) {
        if (isASubConceptOfB(s.second.instanceId.s, "PouringSurface")) {
            index = s.first;
            foundSurface = true;
            break;
        }
    }
    if (!foundSurface) {
        cout << "Can not pour into " << into.instanceId.s << endl;
        return;
    }
    auto intoSurfaceInstance = mapGet(into.parameters->getValue<ObjectConcept::surfacesProperty>().m, index);
    cout << "IntoSurfaceInstance " << intoSurfaceInstance.instanceId.s << ": " << intoSurfaceInstance.geometry.get()
         << endl;
    ObjectSurface *intoSurface = dynamic_pointer_cast<ObjectSurface>(intoSurfaceInstance.geometry).get();
    auto const intoSurfaceInstancePoseVal = GetInstancePose::eval(intoSurfaceInstance);
    cout << "Into Surface Instance pose: " << intoSurfaceInstancePoseVal->q.toString() << endl;
    cout << "Into Surface pose: " << intoSurface->getPose().toString() << endl;
    foundSurface = false;
    for (auto &s: from.parameters->getValue<ObjectConcept::surfacesProperty>()) {
        if (isASubConceptOfB(s.second.instanceId.s, "PouringSurface")) {
            index = s.first;
            foundSurface = true;
        }
    }
    if (!foundSurface) {
        cout << "Can not pour from " << from.instanceId.s << endl;
        return;
    }
    auto fromSurfaceInstance = mapGet(from.parameters->getValue<ObjectConcept::surfacesProperty>().m, index);
    cout << "FromSurfaceInstance " << fromSurfaceInstance.instanceId.s << ": " << fromSurfaceInstance.geometry.get()
         << endl;
    ObjectSurface *fromSurface = dynamic_pointer_cast<ObjectSurface>(fromSurfaceInstance.geometry).get();
    auto const fromSurfaceInstancePoseVal = GetInstancePose::eval(fromSurfaceInstance);
    cout << "From Surface Instance pose: " << fromSurfaceInstancePoseVal->q.toString() << endl;
    cout << "From Surface pose: " << fromSurface->getPose().toString() << endl;

    // for visualization
    if (sim.doesObjectExistInSimulation(fromSurfaceInstance.instanceId.s)) {
        sim.get().set_object_pose(fromSurfaceInstance.instanceId.s, fromPoseToDQ(fromSurfaceInstancePoseVal->q));
    }
    if (sim.doesObjectExistInSimulation(intoSurfaceInstance.instanceId.s)) {
        sim.get().set_object_pose(intoSurfaceInstance.instanceId.s, fromPoseToDQ(intoSurfaceInstancePoseVal->q));
    }

    double const &pourAngle = pourParameters.getValue<PourSkill::angleProperty>().n;
    double const &pourAmount = pourParameters.getValue<PourSkill::amountProperty>().n;
    double const &pourDirection = pourParameters.getValue<PourSkill::directionProperty>().n;
    double const &pourHeight = pourParameters.getValue<PourSkill::heightProperty>().n;
    double const &pourTime = pourParameters.getValue<PourSkill::timeProperty>().n;
    double pourSpinAngle = 0.0;
    if (pourParameters.hasProperty("spinAngle")) {
        pourSpinAngle = pourParameters.getValue<Number>("spinAngle").n;
    }
    cout << "Pour angle = " << pourAngle << endl;
    cout << "Pour amount = " << pourAmount << endl;
    cout << "Pour direction = " << pourDirection << endl;
    cout << "Pour height = " << pourHeight << endl;
    cout << "Pour spin angle = " << pourSpinAngle << endl;
    cout << "Pour time = " << pourTime << endl;

    if (AndreiUtils::equal<double>(pourAmount, 0)) {
        cout << "Pour amount is 0! Don't execute pouring at all :)" << endl;
        return;
    }

    // constants definition: F = from = object from which it is poured, I = into = object into which it is poured
    AndreiUtils::Posed const &I_q_IS = intoSurface->getSurfaceOrigPose();
    AndreiUtils::Posed const &F_q_FS = fromSurface->getSurfaceOrigPose();
    auto const W_q_I = GetInstancePose::eval(into);
    auto const W_q_F = GetInstancePose::eval(from);
    Eigen::Matrix3d I_r_IS = I_q_IS.getRotationAsMatrix();
    Eigen::Matrix3d F_r_FS = F_q_FS.getRotationAsMatrix();
    AndreiUtils::Posed const FS_q_F = F_q_FS.inverse();
    Eigen::Vector3d I_z_IS = I_r_IS * zAxis3d<double>();
    Eigen::Vector3d F_z_FS = F_r_FS * zAxis3d<double>();

    AndreiUtils::Posed goalPose;
    Vector3d direction = Vector3d(cos(pourDirection), sin(pourDirection), 0);
    double l = into.parameters->getValue<ObjectConcept::interactionVolumeProperty>().n;  // bowl interaction volume
    cout << "Into interaction volume = " << l << endl;
    Eigen::Vector3d I_pourDirection = I_q_IS.getRotationAsMatrix() * direction;
    Vector3d I_p_F = I_pourDirection * l * 1.25;

    // bring "from" into interaction volume of "into" along the pouring direction
    AndreiUtils::Posed qTransform1(qIdentity<double>(), F_q_FS.getTranslation().z() * I_z_IS);
    AndreiUtils::Posed qTransform2(Eigen::Quaterniond(Eigen::AngleAxisd(
                                           -AndreiUtils::vectorAngle(AndreiUtils::zAxis3d<double>(), F_z_FS), yAxis3d<double>())),
                                   Eigen::Vector3d::Zero());
    AndreiUtils::Posed qTransform = qTransform1 * qTransform2;
    Eigen::Vector3d I_pourRotationAxis = I_z_IS.cross(I_pourDirection);
    Eigen::Matrix3d I_r0_FS = AndreiUtils::getOrientationFromAxes(I_pourDirection, I_pourRotationAxis, I_z_IS);
    goalPose = W_q_I->q * AndreiUtils::Posed(I_r0_FS, I_p_F) * qTransform * FS_q_F;
    moveObjectToGoal(goalPose, from, path, robot, endEffectorToObjectOriginDQ, 500);

    // raise "from" above "into"
    Eigen::Vector3d pTmp1_FS = I_p_F + (pourHeight + l / 2) * I_z_IS;
    AndreiUtils::Posed I_q_FS_beforeGoingAbovePourPoint = AndreiUtils::Posed(I_r0_FS, pTmp1_FS) * qTransform;
    goalPose = W_q_I->q * I_q_FS_beforeGoingAbovePourPoint * FS_q_F;
    moveObjectToGoal(goalPose, from, path, robot, endEffectorToObjectOriginDQ, 500);

    // determine pour point
    double minAxisSizeHalf = 0;
    for (long i = 0; i < intoSurface->getSurfacePointCount() - 1; ++i) {
        double axisSize = intoSurface->getSurfaceMatrixOrig().col(i).norm();
        cout << "axis size at axis " << i << " = " << axisSize << endl;
        if (i == 0 || axisSize / 2 < minAxisSizeHalf) {
            minAxisSizeHalf = axisSize / 2;
        }
    }
    cout << "MinAxisSize = " << minAxisSizeHalf << endl;
    Eigen::Vector3d I_pourPoint = I_q_IS.getTranslation() + I_pourDirection * minAxisSizeHalf * 0.75;
    cout << "Pour point " << I_pourPoint.transpose() << " vs. surface origin " << I_q_IS.getTranslation().transpose()
         << endl;

    //*
    // go to point above "into"
    Eigen::Vector3d pTmp2_FS = I_pourPoint + (pourHeight + l / 2) * I_z_IS;
    AndreiUtils::Posed I_q_FS_beforePour = AndreiUtils::Posed(I_r0_FS, pTmp2_FS) * qTransform;
    goalPose = W_q_I->q * I_q_FS_beforePour * FS_q_F;
    moveObjectToGoal(goalPose, from, path, robot, endEffectorToObjectOriginDQ, 1000);
    //*/

    // final goal pose of the pourer-surface coordinate frame
    // angle axis (I^z_IS x direction)
    Eigen::Quaterniond rotation(Eigen::AngleAxisd(-pourAngle, I_pourRotationAxis));
    Eigen::Vector3d zFS = qRotate(rotation, I_z_IS);
    // the rotated vector below can be any vector in the x-y plane... must not necessarily be 'I_pourDirection'
    Eigen::Vector3d xFS = qRotate(rotation, I_pourDirection);
    xFS = qRotate(Eigen::Quaterniond(Eigen::AngleAxisd(pourSpinAngle, zFS)), xFS);
    Eigen::Matrix3d I_r_FS = AndreiUtils::getOrientationFromTwoAxes(zFS, xFS);
    Eigen::Vector3d pFinal_FS = I_pourPoint + pourHeight * I_z_IS;
    AndreiUtils::Posed I_q_FS_duringPour(I_r_FS, pFinal_FS);

    AndreiUtils::Posed finalGoalPose = W_q_I->q * I_q_FS_duringPour * FS_q_F;

    // keep I_z_IS-axis constant while lowering to pour!
    auto res = AndreiUtils::PoseDecoupledInterpolator<double>().compute(
            I_q_FS_beforePour, I_q_FS_duringPour, 6).getResult();
    size_t indexUntilAmountIsReached = 1;
    double pouredAmount = 0;
    AndreiUtils::Timer t;
    for (; indexUntilAmountIsReached < res.size(); ++indexUntilAmountIsReached) {
        auto const &q = res[indexUntilAmountIsReached];
        moveObjectToGoal(W_q_I->q * q * FS_q_F, from, path, robot, endEffectorToObjectOriginDQ);
        pouredAmount = t.measure() / amountToTimeScalingFactor;
        cout << "Poured amount = " << pouredAmount << endl;
        if (pourAmount >= 0 && pouredAmount * 2 >= pourAmount) {
            ++indexUntilAmountIsReached;
            break;
        }
    }

    double pouringTime = AndreiUtils::less(pourAmount, 0.0) ? pourTime : (pourAmount - 2 * pouredAmount) *
                                                                         amountToTimeScalingFactor;
    cout << "Remaining pouring time = " << pouringTime << endl;
    AndreiUtils::sleepMSec(int(pouringTime * 1000));

    // go to point above "into"
    for (size_t i = indexUntilAmountIsReached - 1; i-- > 0;) {
        auto const &q = res[i];
        moveObjectToGoal(W_q_I->q * q * FS_q_F, from, path, robot, endEffectorToObjectOriginDQ);
    }
    AndreiUtils::sleepMSec(500);
}

void runUseRobotToPour(SimulationInterface &sim, nlohmann::json const &config) {
    DQ_VrepInterface &vi = sim.get();
    map<string, string> nameConvertor;
    map<string, Eigen::Vector3d> objectSpecificTranslationAdjustment;
    initializeSimData(nameConvertor, objectSpecificTranslationAdjustment);

    std::map<std::string, AndreiUtils::Posed> initialPoses;
    if (config.contains("sampleInitialPose") && config.at("sampleInitialPose").get<bool>()) {
        // TODO: this solution does not work when the pourer is not z-axis aligned//oriented
        auto milkInitialPose = samplePose(0.2, false, true);
        Vector3d t = milkInitialPose.getTranslation();
        t.z() = 0;
        milkInitialPose = Posed(milkInitialPose.getRotation(), t);
        AndreiUtils::mapEmplace<std::string>(initialPoses, "MilkCartonLidlInstance1", milkInitialPose);
        AndreiUtils::mapEmplace<std::string>(initialPoses, "BowlGreyIkeaInstance",
                                             AndreiUtils::Posed(qzRotation(double01Sampler.sample()),
                                                                Vector3d(0, 0.1, 0)));
        AndreiUtils::mapEmplace<std::string>(initialPoses, "PlasticCupInstance1",
                                             AndreiUtils::Posed(qzRotation(double01Sampler.sample()),
                                                                Vector3d(0.2, 0.0, 0)));
        AndreiUtils::mapEmplace<std::string>(initialPoses, "PlasticCupInstance2",
                                             AndreiUtils::Posed(qzRotation(double01Sampler.sample()),
                                                                Vector3d(0.4, 0.0, 0)));
    } else {
        auto initialPosesConfig = config.at("initialPoses");
        if (initialPosesConfig.is_string()) {
            initialPosesConfig = config.at(initialPosesConfig.get<std::string>());
        }
        initialPoses = initialPosesConfig.get<std::map<std::string, AndreiUtils::Posed>>();
    }
    map<std::string, ObjectInstance> objects;
    for (auto const &objName: initialPoses) {
        auto const &obj = AndreiUtils::mapEmplace(objects, objName.first, ObjectInstance(objName.first))->second;
        SetInstancePose::eval(obj, ConceptLibrary::Pose{objName.second});
        auto const objPoseVal = GetInstancePose::eval(obj);
        cout << obj.instanceId.s << " initial pose: " << objPoseVal->q.toString() << endl;
        setSimulationPose(sim, obj, nameConvertor, objectSpecificTranslationAdjustment);
        if (sim.doesObjectExistInSimulation(obj.instanceId.s)) {
            removeObjectFromGripper(vi, obj.instanceId.s);
        }
    }
    vector<string> objectsToDisappear = {"/TrashCan", "/CerealBox", "/BowlWithGrasp", "/MilkCartonLidlInstance1_Mesh",
                                         "/PourDirectionRelToIntoSurface", "/LeftElbow", "/LeftWrist", "/LeftHand",
                                         "/RightElbow", "/RightWrist", "/RightHand", "/Check1"};
    for (auto &s: objectsToDisappear) {
        if (sim.doesObjectExistInSimulation(s)) {
            makeSimulationObjectDisappear(sim.get(), s);
        }
    }

    string whichRobot = "PandaRobot";
    Robot robot(Robot::createFromType(whichRobot, sim, "PandaGripper"));
    Vector7d robotInitialJoints;
    robotInitialJoints << 0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4;
    robot.setJointValues(robotInitialJoints);

    PathController path;
    ObstacleData empty;

    double amountToTimeScalingFactor = 32;
    if (config.contains("amountToTimeScalingFactor")) {
        amountToTimeScalingFactor = config.at("amountToTimeScalingFactor").get<double>();
    }

    // init finished
    AndreiUtils::sleepMSec(2000);

    auto pouringsConfig = config.at("pourings");
    if (pouringsConfig.is_string()) {
        pouringsConfig = config.at(pouringsConfig.get<std::string>());
    }
    for (auto const &pourConfig: pouringsConfig.get<std::vector<nlohmann::json>>()) {
        ObjectInstance *into = &AndreiUtils::mapGet(objects, pourConfig.at("pourInto").get<string>());
        ObjectInstance *from = &AndreiUtils::mapGet(objects, pourConfig.at("pourFrom").get<string>());

        try {
            AndreiUtils::Posed endEffectorToObjectOrigin;
            graspObject(*from, nameConvertor, sim, path, robot, endEffectorToObjectOrigin, config, 500);
            DQ endEffectorToObjectOriginDQ = fromPoseToDQ(endEffectorToObjectOrigin);

            ConceptParameters pourParameters;
            if (pourConfig.contains("useVariationParameters") && pourConfig.at("useVariationParameters").get<bool>()) {
                auto const &params = pourConfig.at("pourVariationParameters");
                pourParameters.setPropertyValue("angle", params.at("angle").get<Variation<Number>>().getSampledValue());
                pourParameters.setPropertyValue("amount",
                                                params.at("amount").get<Variation<Number>>().getSampledValue());
                pourParameters.setPropertyValue("direction",
                                                params.at("direction").get<Variation<Number>>().getSampledValue());
                pourParameters.setPropertyValue("height",
                                                params.at("height").get<Variation<Number>>().getSampledValue());
                pourParameters.setPropertyValue("time", params.at("time").get<Variation<Number>>().getSampledValue());
                pourParameters.setPropertyValue("spinAngle",
                                                params.at("spinAngle").get<Variation<Number>>().getSampledValue());
            } else {
                auto const &params = pourConfig.at("pourParameters");
                pourParameters.setPropertyValue("angle", params.at("angle").get<Number>());
                pourParameters.setPropertyValue("amount", params.at("amount").get<Number>());
                pourParameters.setPropertyValue("direction", params.at("direction").get<Number>());
                pourParameters.setPropertyValue("height", params.at("height").get<Number>());
                pourParameters.setPropertyValue("time", params.at("time").get<Number>());
                pourParameters.setPropertyValue("spinAngle", params.at("spinAngle").get<Number>());
            }
            pourFromInto(*from, *into, pourParameters, sim, path, robot, endEffectorToObjectOriginDQ,
                         amountToTimeScalingFactor);

            releaseObject(*from, AndreiUtils::mapGet(initialPoses, from->instanceId.s), nameConvertor, sim, path, robot,
                          endEffectorToObjectOriginDQ, 500);

            path.simulationControlToJointValues(&robot, robotInitialJoints, 300);
            AndreiUtils::sleepMSec(500);
        } catch (std::runtime_error &e) {
            if (std::string(e.what()) != "Robot reached workspace limit!") {
                throw e;
            }
            // detach from object
            removeObjectFromGripper(sim.get(), AndreiUtils::mapGet(nameConvertor, from->instanceId.s));
            cout << "Robot reached workspace limit, try to recover to a known position..." << endl;
            path.simulationControlToJointValues(&robot, robotInitialJoints, 300);
        }
    }
}

void useRobotToPour(nlohmann::json const &config) {
    SimulationInterface &sim = SimulationInterface::getInterface();
    try {
        runUseRobotToPour(sim, config);
    } catch (exception &e) {
        cout << "Caught exception: " << e.what() << endl;
    }
}

// this example assumes CoppeliaSim is running and a scene called test_object_origins.ttt is loaded in the simulator
void listObjectsInSimulation() {
    SimulationInterface &s = SimulationInterface::getInterface();
    std::map<std::string, std::string> chInstanceToSimulationNameConvertor = {
            {"MilkCartonLidlInstance1",   "/MilkCartonLidlInstance1"},
            {"trash_can",                 "/TrashCan"},
            {"CerealBoxVitalisInstance1", "/CerealBox"},
            {"BowlGreyIkeaInstance",      "/BowlGreyIkeaInstance"},
           // {"BowlWhiteIkeaInstance",     "/BowlWhiteIkeaInstance"},
            {"PlasticCupInstance1",       "/PlasticCupInstance1"},
            {"PlasticCupInstance2",       "/PlasticCupInstance2"},
    };
    for (auto const &nameData: chInstanceToSimulationNameConvertor) {
        if (!s.doesObjectExistInSimulation(nameData.second)) {
            throw std::runtime_error("Object named " + nameData.second + " does not exist in the scene!");
        }
    }

    auto &simInterface = s.get();
    AndreiUtils::sleepMSec(1000);
    for (auto const &nameData: chInstanceToSimulationNameConvertor) {
        AndreiUtils::Posed q = fromDQToPose(simInterface.get_object_pose(nameData.second));  // get pose in simulation
        q = q.addTranslation(Eigen::Vector3d(0.1, 0.075, 0.05));  // move the object a bit
        simInterface.set_object_pose(nameData.second, fromPoseToDQ(q));  // update the pose in simulation
        AndreiUtils::sleepMSec(1000);  // wait to observe the result
    }
    AndreiUtils::sleepMSec(2000);  // sleep so that the final result is still visible before the simulation stops
}

// this example assumes CoppeliaSim is running and a scene called test_object_origins.ttt is loaded in the simulator
void testMoveRobot() {
    SimulationInterface &sim = SimulationInterface::getInterface();
    string whichRobot = "PandaRobot";
    Robot robot(Robot::createFromType(whichRobot, sim, "PandaGripper"));
    Vector7d robotInitialJoints;
    robotInitialJoints << 0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4;  // update joint values
    robot.setJointValues(robotInitialJoints);  // transmit joint values to robot
    AndreiUtils::sleepMSec(1000);  // update joint values
    robotInitialJoints << 0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, (M_PI_4 + 0.1);  // update joint values
    robot.setJointValues(robotInitialJoints);  // transmit joint values to robot
    AndreiUtils::sleepMSec(1000);  // update joint values
    robotInitialJoints << 0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, (M_PI_4 + 0.2);  // update joint values
    robot.setJointValues(robotInitialJoints);  // transmit joint values to robot
    AndreiUtils::sleepMSec(1000);  // update joint values
    robotInitialJoints << 0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, (M_PI_4 + 0.3);  // update joint values
    robot.setJointValues(robotInitialJoints);  // transmit joint values to robot
    AndreiUtils::sleepMSec(1000);  // update joint values
    robotInitialJoints << 0.3, (-M_PI_4 + 0.3), 0.3, (-3 * M_PI_4 + 0.3), 0.3, (M_PI_2 + 0.3), (M_PI_4 + 0.3);  // update joint values
    robot.setJointValues(robotInitialJoints);  // transmit joint values to robot
    AndreiUtils::sleepMSec(1000);  // update joint values

    PathController path;
    // smooth control in simulation from current to goal joint configuration (which in this case is called robotInitialJoints)
    path.simulationControlToJointValues(&robot, robotInitialJoints, 300);
    cout << "Done moving robot!" << endl;
    AndreiUtils::sleepMSec(2000);  // sleep so that the final result is still visible before the simulation stops
    path.simulationControlToDestination(&robot, fromPoseToDQ(AndreiUtils::Posed{AndreiUtils::qxRotation(M_PI), Eigen::Vector3d{0, 0.1, 0.05}}));
    AndreiUtils::sleepMSec(2000);  // sleep so that the final result is still visible before the simulation stops
}

void testIKsolver() {
    SimulationInterface &sim = SimulationInterface::getInterface();
    string whichRobot = "PandaRobot";
    Robot robot(Robot::createFromType(whichRobot, sim, "PandaGripper"));
    Vector7d robotInitialJoints;
    robotInitialJoints << 0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4;  // update joint values
    robot.setJointValues(robotInitialJoints);  // transmit joint values to robot
    AndreiUtils::sleepMSec(1000);  // update joint values
    robotInitialJoints << 0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, (M_PI_4 + 0.1);  // update joint values
    robot.setJointValues(robotInitialJoints);  // transmit joint values to robot

    PathController path;


    robotInitialJoints << 1.12094, 1.17353,-0.109308,-2.12641,-0.576416,0.185604,1.49456;  // from IK solver, when -45 is not included
    path.simulationControlToJointValues(&robot, robotInitialJoints, 300);
    AndreiUtils::sleepMSec(1000);
    cout << "Done testing IK solution1!" << endl;
    //robotInitialJoints << -1.51933, -1.25395, 2.76818, -2.24399, -0.842722, 0.48578, 1.64371; // from IK solver, when -45 is included
    robotInitialJoints << -0.601442, 1.7628, 0.160531, -1.58072, 0.67413, 0.262201, -2.8973;// from IK solver with base transform, going perfecly to global zero




    // smooth control in simulation from current to goal joint configuration (which in this case is called robotInitialJoints)
    path.simulationControlToJointValues(&robot, robotInitialJoints, 300);
    cout << "Done testing IK solution2 with base transform!" << endl;
    AndreiUtils::sleepMSec(2000);  // sleep so that the final result is still visible before the simulation stops
    auto ikGoal = AndreiUtils::DualQuaternion<double>::createFromCoefficients(1,0,0,0,0.0, 0.0, 0.0, 0.0);

    path.simulationControlToDestination(&robot, fromPoseToDQ(ikGoal) );
    cout << "Done testing IK goal!" << endl;
    AndreiUtils::sleepMSec(8000);
}

class MotionPrimitiveParam {
public:
    virtual ~MotionPrimitiveParam() = default;
};

class MoveRobotBodyCartesianParam : public MotionPrimitiveParam {
public:
    AndreiUtils::Pose goalPose;

    explicit MoveRobotBodyCartesianParam(AndreiUtils::Pose pose) : MotionPrimitiveParam(), goalPose(std::move(pose)) {}
};

class MoveGripperParam : public MotionPrimitiveParam {
public:
    bool openGripper;

    explicit MoveGripperParam(bool isOpen) : MotionPrimitiveParam(), openGripper(isOpen) {}
};

class SetObjectInGripperParam : public MotionPrimitiveParam {
public:
    InstanceAccept<ObjectConcept> o;

    explicit SetObjectInGripperParam(Instance<ConceptList<ObjectConcept>> const &objectInstance) :
            MotionPrimitiveParam(), o(objectInstance) {}
};

/*
class Execution {
public:
    // Constructor initialized once
    Execution() :
          sim(SimulationInterface::getInterface()), robot(Robot::createFromType("PandaRobot", sim, "PandaGripper")),
          path() {
        initializeSimData(nameConvertor, objectSpecificTranslationAdjustment);
    }

    // Method to execute a single motion primitive
    void executability(std::shared_ptr<MotionPrimitiveParam> const &primitive) {
        if (auto moveBody = std::dynamic_pointer_cast<MoveRobotBodyCartesianParam>(primitive)) {
            executeMoveRobotBodyCartesian(*moveBody);
        } else if (auto gripper = std::dynamic_pointer_cast<MoveGripperParam>(primitive)) {
            std::cout << "[Execution] Executing MoveGripper → "
                      << "Open: " << std::boolalpha << gripper->openGripper << "\n";
        } else if (auto setObj = std::dynamic_pointer_cast<SetObjectInGripperParam>(primitive)) {
            executeSetObjectInGripper(*setObj);
        } else {
            std::cout << "[Execution] Unknown motion primitive type\n";
        }
    }

private:
    //  objects and initialization
    SimulationInterface& sim;
    Robot robot;
    PathController path;

    // one-time initialized nameConvertor
    std::map<std::string, std::string> nameConvertor;
    std::map<std::string, Eigen::Vector3d> objectSpecificTranslationAdjustment;

    // internal executors for specific motion primitives
    void executeMoveRobotBodyCartesian( MoveRobotBodyCartesianParam const &param) {
        cout << "moving robot body in cartesian!" << endl;
        path.simulationControlToDestination(&robot, fromPoseToDQ(param.goalPose));
        AndreiUtils::sleepMSec(1000);
    }

    void executeSetObjectInGripper(SetObjectInGripperParam const &param) {
        cout << "setting (linking) object in gripper!" << endl;
        Gripper* gripper = robot.getGripper();
        std::string const &instanceId = param.o.instanceId.s;
        std::string const &simObjectName = AndreiUtils::mapGet(nameConvertor, instanceId);

        sim.get().set_object_parent(simObjectName, gripper->getGripperName(), true);
        AndreiUtils::sleepMSec(1000);
    }
};
*/
class Execution : public ConceptLibrary::Executor {
public:
    Execution()
        : Executor(),
          sim(&SimulationInterface::getInterface()),
          robot(Robot::createFromType("PandaRobotWithGripper", *sim, "PandaGripper")),
          path() {
        initializeSimData(nameConvertor, objectSpecificTranslationAdjustment);
        // Register abilities that this class can handle
        addAbility("MoveRobotBodyCartesianWithIntermediateGoals");
        addAbility("MoveRobotBodyCartesian");
        addAbility("MoveGripper");
        addAbility("IdleAgent");
        addAbility("SetObjectInGripper");
        addAbility("SeeThenMoveToObject");
        addAbility("ClearObjectInGripper");
        addAbility("UpdateProprioception");
        addAbility("LocalizeObject");

    }
    using ValueDomain::compare;

    Boolean compare(ValueDomain const *other, InstanceCacheData &memory) const override {
        auto const cast = dynamic_cast<Execution const *>(other);
        if (cast == nullptr) {
            return falseBoolean;
        }
        if (&(this->robot) != &(cast->robot)) {
            return falseBoolean;
        }
        if (&(this->path) != &(cast->path)) {
            return falseBoolean;
        }
        if (this->nameConvertor != cast->nameConvertor) {
            return falseBoolean;
        }
        if (this->objectSpecificTranslationAdjustment != cast->objectSpecificTranslationAdjustment) {
            return falseBoolean;
        }
        if (&(this->sim) != &(cast->sim)) {
            return falseBoolean;
        }
        // If all checks passed
        return trueBoolean;
    }

    std::shared_ptr<Concept> copy() const override {
        return std::make_shared<Execution>(*this);
    }

    using ValueDomain::clone;

    std::shared_ptr<ValueDomain> clone(ValueDomainDeserializerParameters &memory) const override {
        return this->cloneValueFunction<GeometricShape>(memory);
    }

    using ValueDomain::getStringRepresentation;

    std::string getStringRepresentation(bool spread, int intent, InstanceCacheData &memory) const override {
        return this->surroundRepresentationWithType("[Execution]");
    }

    void initializeEnvironmentFromSimulation(EnvironmentData const &envData) const {
        // in the function, go through all the entities of the environment and if they are contained in simulation, get their pose and update their pose with SetInstancePose
        std::vector<std::string> presentObjectsInEnv;
        auto &simInterface = sim->get();

        for (const auto &pair : nameConvertor) {  // this for loop is for graspable objects
            const std::string &objectName = pair.first;
            const std::string &simObjectName = pair.second;

            // Check if the object exists in the simulation first
            if (!sim->doesObjectExistInSimulation(simObjectName)) {
                continue; // Skip if the object is not in the simulation
            }

            if (!envData.hasEntity(InstanceAccept<ObjectConcept>(objectName).instanceId)) {
                continue;
            }
            // Get pose from simulation
            AndreiUtils::Posed poseInSim = fromDQToPose(simInterface.get_object_pose(simObjectName));
            ConceptLibrary::Pose rawPose(poseInSim);
            // Get object instance from env
            auto objectInstance = envData.getEntity(InstanceAccept<ObjectConcept>(objectName).instanceId);
            // Set the pose into the environment
            SetInstancePose::eval(objectInstance, rawPose);
        }
        auto const franka= envData.getEntity("FrankaPanda");
        auto const &gripper = AndreiUtils::mapGet<String>(franka.parameters->getValue<AgentConcept::grippersProperty>().m, "FrankaPanda_FrankaGripper");

        if (sim->doesObjectExistInSimulation("/Franka/FrankaGripper")&& envData.hasEntity(gripper.instanceId )) {
            // Get pose from simulation
            cout<< "-> Gripper both exist in simulation and envData. Initializing gripper."<< endl;
            AndreiUtils::Posed poseInSim = fromDQToPose(simInterface.get_object_pose("/Franka/FrankaGripper"));
            ConceptLibrary::Pose rawPose(poseInSim);
            // Get object instance from env
            auto objectInstance = envData.getEntity(gripper.instanceId);
            // Set the pose into the environment
            SetInstancePose::eval(objectInstance, rawPose);

        }
        // Checking whether the robot exists in both the sim and env. TODO: add this to loop as well.
        if (sim->doesObjectExistInSimulation("/Franka")&& envData.hasEntity(franka.instanceId )) {
            // Get pose from simulation
            cout<< "-> Franka both exist in simulation and envData. Initializing Franka."<< endl;
            AndreiUtils::Posed poseInSim = fromDQToPose(simInterface.get_object_pose("/Franka"));
            ConceptLibrary::Pose rawPose(poseInSim);

            // Get object instance from env
            auto objectInstance = envData.getEntity(franka.instanceId);

            // Set the pose into the environment
            SetInstancePose::eval(objectInstance, rawPose);
        }
    }

    DualQuaternion<double> getCurrentEEPoseOfRobot(){
        return fromDQToPose(robot.getCurrentRobotEEPose());
    }

    bool executeAbility(InstanceAccept<AbilityConcept> const &ability, EnvironmentData &env) override {  // the main function which executes the motion primitives

        if (ability.isSubConceptOfNoCheck("MoveRobotBodyCartesian") && !ability.isSubConceptOfNoCheck("MoveRobotBodyCartesianWithIntermediateGoals")) {
            auto const &goalLocation = ability.parameters->getValue<MoveRobotBodyCartesianAbility::goalProperty>();
            auto const &goalPose= goalLocation.getGlobalPose();
            auto const goalPosed = goalPose.q;
            executeMoveRobotBodyCartesian(goalPosed);
        } else if (ability.isSubConceptOfNoCheck("MoveRobotBodyCartesianWithIntermediateGoals")) {
            executeMoveRobotBodyCartesianWithIntermediateGoals(ability);
        } else if (ability.isSubConceptOfNoCheck("LocalizeObject")) {
            executeLocalizeObject(ability, env);
        } else if (ability.isSubConceptOfNoCheck("IdleAgent")) {
            executeIdleAgent(ability);
        } else if (ability.isSubConceptOfNoCheck("MoveGripper")) {
            auto const gripperStatusOpen= ability.parameters->getValue<MoveGripperAbility::openGripperProperty>();
            executeMoveGripper(gripperStatusOpen);
        } else if (ability.isSubConceptOfNoCheck("SetObjectInGripper")) {
            auto const &object = ability.parameters->getValue<SetObjectInGripperAbility::oProperty>();
            executeSetObjectInGripper(object);
        } else if (ability.isSubConceptOfNoCheck("ClearObjectInGripper")) {
            auto const &object = ability.parameters->getValue<ClearObjectInGripperAbility::oProperty>();
            executeClearObjectInGripper(object);
        } else if (ability.isSubConceptOfNoCheck("UpdateProprioception")) {
            executeUpdateProprioception(env);
        } else if (ability.isSubConceptOfNoCheck("SeeThenMoveToObject")) {

            if (!ability.parameters->hasKnownProperty("ignoreInstancesOfConcepts")) {
                ability.parameters->setPropertyValue<Sequence<ConceptValue>>("ignoreInstancesOfConcepts", Sequence<ConceptValue>{});
            }
            auto const &conceptVal = ability.parameters->getValue<SeeThenMoveToObjectAbility::objectConceptToGoToProperty>();
            auto const &deltaPose = ability.parameters->getValue<SeeThenMoveToObjectAbility::deltaPoseToObjectProperty>();
            auto const &ignoreInstances = ability.parameters->getValue<SeeThenMoveToObjectAbility::ignoreInstancesOfConceptsProperty>();
            //auto const &useCartesian = ability.parameters->getValue<SeeThenMoveToObjectAbility::useCartesianProperty>();
            //auto const &waitTimeSec = ability.parameters->getValue<SeeThenMoveToObjectAbility::waitTimeSecProperty>();

            Instance<ConceptList<EntityConcept>> goalObjectInstanceSeeThenMove = executeSeeThenMoveToObject(conceptVal,env, deltaPose ,ignoreInstances);
            ability.parameters->setPropertyValue<SeeThenMoveToObjectAbility::whichObjectProperty>(goalObjectInstanceSeeThenMove);
        } else {
            std::cout << "[Execution] Unknown ability name: " << ability.instanceId.s << std::endl;
            return false;

        }

        return true;
    }

protected:
    void cloneValueDataTo(std::shared_ptr<ValueDomain> &copy, ValueDomainDeserializerParameters &memory) const override {
        auto const localCopy = std::dynamic_pointer_cast<Execution>(copy);
        if (localCopy == nullptr) {
            return;
        }
        this->ValueDomain::cloneValueDataTo(copy, memory);
    }

private:
    SimulationInterface *sim;
    Robot robot;
    PathController path;
    std::map<std::string, std::string> nameConvertor;
    std::map<std::string, Eigen::Vector3d> objectSpecificTranslationAdjustment;

    void executeMoveRobotBodyCartesian(AndreiUtils::Pose  const &goalPose) {

        std::cout << "Executing: MoveRobotBodyCartesian\n";
        path.simulationControlToDestination(&robot, fromPoseToDQ(goalPose));
        //AndreiUtils::sleepMSec(1000); // MAKE THIS WAIT OPTIONAL
    }
    void executeMoveRobotBodyCartesianWithIntermediateGoals(InstanceAccept<AbilityConcept> const &ability){

        std::cout << "Executing: MoveRobotBodyCartesianWithIntermediateGoals\n";
        auto const isGoalIncluded=ability.parameters->getValue<MoveRobotBodyCartesianWithIntermediateGoalsAbility::isGoalIncludedProperty>();
        auto  waypoints=ability.parameters->getValue<MoveRobotBodyCartesianWithIntermediateGoalsAbility::intermediateGoalsProperty>().seq;
        auto const timeout=ability.parameters->getValue<MoveRobotBodyCartesianWithIntermediateGoalsAbility::timeoutProperty>();
        cout<< "Timeout value for the MoveRobotBodyCartesianWithIntermediateGoals is " << timeout.n << endl;

        if(!isGoalIncluded.b){
            waypoints.pop_back();
        }

        int waypointIndexUntilTimeout = 0;
        double elapsedTime = 0;
        AndreiUtils::Timer t;

        for (; waypointIndexUntilTimeout < waypoints.size(); ++waypointIndexUntilTimeout) {
            auto goalLocation = waypoints[waypointIndexUntilTimeout];
            auto goalPose =goalLocation.getGlobalPose().q;
            executeMoveRobotBodyCartesian(goalPose);
            elapsedTime = t.measure();
            if (timeout.n >= 0 && elapsedTime >= timeout.n) {
                cout<<"Timeout reached !"<< endl;
                ++waypointIndexUntilTimeout;
                ability.parameters->setPropertyValue<MoveRobotBodyCartesianWithIntermediateGoalsAbility::isTimeoutReachedProperty>(trueBoolean);
                break;
            }
        }

        ability.parameters->setPropertyValue<MoveRobotBodyCartesianWithIntermediateGoalsAbility::waypointIndexUntilTimeoutReachedProperty>(Number(waypointIndexUntilTimeout));
        ability.parameters->setPropertyValue<MoveRobotBodyCartesianWithIntermediateGoalsAbility::elapsedTimeProperty>(Number(elapsedTime));
    }

    void executeLocalizeObject(InstanceAccept<AbilityConcept> const &ability, EnvironmentData const &envData){
        cout<< "Executing LocalizeObject " <<endl;
        auto const conceptValue=ability.parameters->getValue<LocalizeObjectAbility::objectConceptToLocalizeProperty>();

        auto &simInterface = sim->get();

        for (const auto &pair : nameConvertor) {  // this for loop is for graspable objects
            const std::string &objectName = pair.first;
            const std::string &simObjectName = pair.second;

            // Check if the object exists in the simulation first
            if (!sim->doesObjectExistInSimulation(simObjectName)) {
                continue; // Skip if the object is not in the simulation
            }

            if (!envData.hasEntity(InstanceAccept<ObjectConcept>(objectName).instanceId)) {
                continue;
            }

            if (!InstanceAccept<ObjectConcept>(objectName).isSubConceptOfNoCheck(conceptValue.s)){ // skip if object is not suitable for Localization
                continue;
            }
            cout<< "Localizing object: " << objectName << endl;
            // Get pose from simulation
            AndreiUtils::Posed poseInSim = fromDQToPose(simInterface.get_object_pose(simObjectName));
            ConceptLibrary::Pose rawPose(poseInSim);
            // Get object instance from env
            auto objectInstance = envData.getEntity(InstanceAccept<ObjectConcept>(objectName).instanceId);
            // Set the pose into the environment
            SetInstancePose::eval(objectInstance, rawPose);
        }
    }

    void executeIdleAgent(InstanceAccept<AbilityConcept> const &ability){
        auto const waitingTime= ability.parameters->getValue<IdleAgentAbility::waitTimeSecProperty>();
        std::cout << "Executing Idle Agent with "<< waitingTime.n << "seconds"<< endl;
        AndreiUtils::sleepMSec(int(waitingTime.n * 1000));
    }

    void executeMoveGripper(Boolean const gripperStatusOpen) {
        if (gripperStatusOpen.b) {
            cout<< "Opening gripper" << endl;
        }
        else {
            cout<< "Closing gripper" << endl;
        }

        AndreiUtils::sleepMSec(1000);
    }
    Instance<ConceptList<EntityConcept>> executeSeeThenMoveToObject( ConceptValue const &conceptValue, EnvironmentData &env, ConceptLibrary::Pose const &deltaPose= ConceptLibrary::Pose(), Sequence<ConceptValue> const & ignoreInstances= {}, Boolean const &useCartesian= trueBoolean, Number const &waitTimeSec= Number(1.0)) {

        std::cout << "Executing SeeThenMoveToObject\n";
        Instance<ConceptList<EntityConcept>> goalObjectInstance;
        auto currentEEPose =  fromDQToPose(robot.getCurrentRobotEEPose());
        auto currentEEXYZ= currentEEPose.getTranslation();

        // 2) Among entities satisfying conceptValue.s, pick the closest (second pass)
        double bestDist2 = std::numeric_limits<double>::infinity();
        auto closestIt   = env.entities.end();
        for (auto it = env.entities.begin(); it != env.entities.end(); ++it) {
            auto &instanceInEnv = *it;

            // Skip if in ignoreInstances (either name match or subconcept match)
            bool skip = false;
            for (const auto& cv : ignoreInstances) {
                if (instanceInEnv.second.isSubConceptOfNoCheck(cv.s)) {
                    cout<< "Skipping " << instanceInEnv.second.instanceId.s <<" because it belongs to " << cv.s <<" in the IgnoreInstances"<< endl;
                    skip = true;
                    break;
                }
            }
            if (skip) continue;

            if (instanceInEnv.second.isSubConceptOfNoCheck(conceptValue.s)) {
                const auto instanceXYZ =
                        instanceInEnv.second.parameters
                                ->getValue<GraspableObjectConcept::locationProperty>()
                                .getGlobalPose().q.getTranslation();

                const double d2 = (instanceXYZ - currentEEXYZ).squaredNorm();
                if (d2 < bestDist2) {
                    bestDist2 = d2;
                    closestIt = it;
                }
            }
        }

        if (closestIt != env.entities.end()) {
            std::cout << "Closest suitable ("<< conceptValue.s<<") object for SeeThenMoveToObject: " << closestIt->first.s
                      << " | distance to EE: " << std::sqrt(bestDist2) << "\n";

            // Extract the full pose from the iterator
            goalObjectInstance = closestIt->second;
            auto const goalPosedForTheFoundClosestObject = closestIt->second.parameters->getValue<ObjectConcept::locationProperty>().getGlobalPose().q;

            //Computing the goal Pose by adding the deltaPose to Object's Pose
            auto const goalPosedWithDeltaPoseToObjectAdded= goalPosedForTheFoundClosestObject * deltaPose.q;

            if(useCartesian) {
                cout<<"Going to the closest suitable object "<<closestIt->first.s<<" by using Cartesian" << endl;
                executeMoveRobotBodyCartesian(goalPosedWithDeltaPoseToObjectAdded);

            }
            else {
                cout<<"Going to the closest suitable object by using another method different than Cartesian is not yet implemented!"<< endl;
            };

        } else {
            std::cout << "No suitable objects found for concept: " << conceptValue.s << "\n";
        }

        AndreiUtils::sleepMSec(waitTimeSec.n*2000);

        return goalObjectInstance;
    }

    void executeSetObjectInGripper(const Instance<ConceptList<ObjectConcept>> object) {
        std::cout << "Executing SetObjectInGripper\n";
        Gripper* gripper = robot.getGripper();
        std::string const &instanceId = object.instanceId.s;
        std::string const &simObjectName = AndreiUtils::mapGet(nameConvertor, instanceId);
        sim->get().set_object_parent(simObjectName, gripper->getGripperName(), true);
        AndreiUtils::sleepMSec(1000);
    }
    void executeClearObjectInGripper(const Instance<ConceptList<ObjectConcept>> object) {
        std::cout << "Executing ClearObjectInGripper\n";
        std::string const &instanceId = object.instanceId.s;
        std::string const &simObjectName = AndreiUtils::mapGet(nameConvertor, instanceId);
        removeObjectFromGripper(sim->get(), simObjectName);
        AndreiUtils::sleepMSec(1000);
    }
    void executeUpdateProprioception(EnvironmentData &env){
        // updating gripper Pose to be the same with EE pose
        std::cout << "Executing UpdateProprioception. Updating gripper Pose. \n";
        auto currentEElocation = getCurrentEEPoseOfRobot();
        auto const franka= env.getEntity("FrankaPanda");
        auto const &gripper = AndreiUtils::mapGet<String>(franka.parameters->getValue<AgentConcept::grippersProperty>().m, "FrankaPanda_FrankaGripper");
        SetInstancePose::eval(gripper, ConceptLibrary::Pose {currentEElocation});
    }
};

void testMotionPrimitiveExecution() {
    cout << "Testing Motion Primitive Executions!" << endl;
    AndreiUtils::Pose someGoalPose;
    AndreiUtils::Pose someGoalPose2;
    someGoalPose = samplePose(0.2, false, true);
    someGoalPose2 = samplePose(0.2, false, true);
    bool shouldOpen = true;
    InstanceAccept<ObjectConcept> graspableObjInstance_milkcarton("MilkCartonLidlInstance1");
    InstanceAccept<ObjectConcept> graspableObjInstance_plasticcup1("PlasticCupInstance1");
    InstanceAccept<ObjectConcept> graspableObjInstance_plasticcup2("PlasticCupInstance2");
    InstanceAccept<GraspableObjectConcept> graspableObjInstance_BowlGrey("BowlGreyIkeaInstance");
    InstanceAccept<EntityConcept> groundInstance("GroundInstance");



    Execution exec;

    //exec.path.simulationControlToDestination(&exec.robot, fromPoseToDQ(AndreiUtils::Posed{AndreiUtils::qxRotation(M_PI), Eigen::Vector3d{0, 0.1, 0.05}}));
    //AndreiUtils::sleepMSec(10000);
    //return;

    EnvironmentData envData;

    InstanceAccept<EntityWithExecutorConcept> franka("FrankaPanda");  // before it was this

    //Instance<ConceptList<AgentConcept>> franka("FrankaPanda");

    // franka.parameters->setPropertyValue<ConceptLibrary::EntityWithExecutorConcept::executorProperty::type>("executor", Value<Executor>{exec});
    franka.parameters->setPropertyValue<ConceptLibrary::EntityWithExecutorConcept::executorProperty::type>("executor", exec);  // before converting it to  agent concept
    //franka.parameters->setPropertyValue<ConceptLibrary::EntityWithExecutorConcept::executorProperty::type>("executor", Value<Executor>{exec});

    // before this function, initialize all the entity-instances in the environment with AddObject- and AddAgentToEnvironment(envData, );



    AddAgentToEnvironment::eval(envData, InstanceAccept<AgentConcept>{franka});
    AddObjectToEnvironment::eval(envData, graspableObjInstance_milkcarton );
    AddObjectToEnvironment::eval(envData, graspableObjInstance_plasticcup1 );
    AddObjectToEnvironment::eval(envData, graspableObjInstance_plasticcup2 );
    AddObjectToEnvironment::eval(envData, InstanceAccept<ObjectConcept>{graspableObjInstance_BowlGrey} );
    AddObjectToEnvironment::eval(envData,InstanceAccept<ObjectConcept>{groundInstance});
    exec.initializeEnvironmentFromSimulation(envData); // setting current locations

    // some tests for grasp location
    auto const testpose= graspableObjInstance_plasticcup2.parameters->getValue<GraspableObjectConcept::locationProperty>().getGlobalPose();

    auto const testdqzero= AndreiUtils::DualQuaternion<double>::createFromCoefficients(0,0,0,1,0.0, 0.0, 0.0, 0.0);



    auto const &gripper = AndreiUtils::mapGet<String>(franka.parameters->getValue<AgentConcept::grippersProperty>().m, "FrankaPanda_FrankaGripper");
    auto const &gripperLocation = gripper.parameters->getValue<PhysicalEntityConcept::locationProperty>();

    ConceptLibrary::Pose zeroPose(testdqzero);
    auto const entitiesInsideEnv= envData.entities;


    auto const agent = InstanceAccept<AgentConcept>{franka};

    auto const &bowlLocation = graspableObjInstance_BowlGrey.parameters->getValue<PhysicalEntityConcept::locationProperty>();
    cout << bowlLocation.getGlobalPose().q.toString(true) << endl;
    cout << bowlLocation.getStringRepresentation(true) << endl;

    auto const graspLocation= DetermineGraspLocation::eval(agent, gripper, graspableObjInstance_BowlGrey );
    if (graspLocation->isEmpty()) {
        cout << "No grasp location found!" << endl;
        return;
    }
    cout << "Determined location: " << graspLocation.getStringRepresentation() << endl;
    auto const graspPosetest= graspLocation.get().getGlobalPose();
    //auto const deltaPoseToObject = exec.computeDeltaPoseForPreGrasp(graspPosetest,graspableObjInstance_BowlGrey);
    auto const graspXYZ= graspLocation.get().getGlobalPose().q.getTranslation();
    auto const graspRotAngles= graspLocation.get().getGlobalPose().q.getRotation();

    auto const goalForBowl= ConceptLibrary::Pose(AndreiUtils::Posed {graspRotAngles, Eigen::Vector3d{0.2, 0.10, 0.3}});
    auto const testformismatch= ConceptLibrary::Pose(AndreiUtils::Posed { Eigen::Quaterniond{0,1,0,0}, Eigen::Vector3d{0.2, 0.20, 0.2}});


    cout<< "grasp Location for the Bowl cup Instance is: "<< graspXYZ<< endl;
    cout<< "grasp Rotation Quaternion for the Bowl cup Instance is: "<< graspRotAngles<< endl;








    // in the function, go through all the entities of the environment and if they are contained in simulation, get their pose and update their pose with SetInstancePose

    /*

    std::vector<std::string> foundObjects = exec.initializeEnvironmentFromSimulation(envData);
    for (const auto& name : foundObjects) {
        std::cout << "Found in environment: " << name << std::endl;
    }
    */




    ConceptParameters instanceProperties;
    instanceProperties.setPropertyValue<Location>("goal", Location{testformismatch});  // or zeroPose for test or graspPosetest for grasp location test
    instanceProperties.setPropertyValue<InstanceAccept<AgentConcept>>("executor", franka);
    InstanceAccept<AbilityConcept> moveRobot("testMoveRobotCartesian1", {"MoveRobotBodyCartesian"}, instanceProperties);

    ConceptParameters instanceProperties_set;
    instanceProperties_set.setPropertyValue<Instance<ConceptList<ObjectConcept>>>("o", Instance<ConceptList<ObjectConcept>>{graspableObjInstance_BowlGrey});
    instanceProperties_set.setPropertyValue<InstanceAccept<AgentConcept>>("executor", franka);
    InstanceAccept<AbilityConcept> setObject("testSetObject",{"SetObjectInGripper"}, instanceProperties_set );

    ConceptParameters instanceProperties_last;
    instanceProperties_last.setPropertyValue<Location>("goal", Location{goalForBowl});
    instanceProperties_last.setPropertyValue<InstanceAccept<AgentConcept>>("executor", franka);
    InstanceAccept<AbilityConcept> moveRobot2("testMoveRobotCartesian2", {"MoveRobotBodyCartesian"}, instanceProperties_last);



    auto const conceptVal = ConceptValue("BowlGreyIkeaInstance");  // Or try directly the instance name :  BowlGreyIkeaInstance  Or try directly the ConceptValue name : "GraspableObject
    Sequence<ConceptValue> ignoreThese(std::vector<ConceptValue>{ ConceptValue("OpenableObject")});
    ConceptLibrary::Pose deltaPose(AndreiUtils::Posed{AndreiUtils::qxRotation(M_PI), Eigen::Vector3d{0, 0.2, 0}});

    ConceptParameters instancePropertiesSeeThenMove;
    instancePropertiesSeeThenMove.setPropertyValue<ConceptValue>("objectConceptToGoTo", conceptVal );
    instancePropertiesSeeThenMove.setPropertyValue<InstanceAccept<AgentConcept>>("executor", franka);
    //instancePropertiesSeeThenMove.setPropertyValue<Sequence<ConceptValue>>("ignoreInstancesOfConcepts", ignoreThese);
    //instancePropertiesSeeThenMove.setPropertyValue<ConceptLibrary::Pose>("deltaPoseToObject", deltaPoseToObject);   // commented this out because computeDeltaPoseToObject function is not available in Execution class anymore
    InstanceAccept<AbilityConcept> seeThenMove("testSeeThenMoveToObject", {"SeeThenMoveToObject"}, instancePropertiesSeeThenMove);




    /*
    for ( auto const &primitive : motionPrimitives) {
        exec.executability(primitive);  // reuse the same Execution
    }
    //*/
    //auto res4 =ExecuteAbility::eval(franka, seeThenMove,envData);
    auto res = ExecuteAbility::eval(franka, moveRobot,envData);
    auto currentEElocation = exec.getCurrentEEPoseOfRobot();
    exec.initializeEnvironmentFromSimulation(envData);
    auto currentGripperLocation= GetInstancePose::eval(envData.getEntity(gripper.instanceId));
    cout << "current EE location of the robot is: "<< currentEElocation.toString()<< endl;
    cout << "current gripper location  is: "<< currentGripperLocation->q.toString()<< endl;
    AndreiUtils::sleepMSec(10000);
    //auto res2 = ExecuteAbility::eval(franka, setObject,envData);
    //auto res3 = ExecuteAbility::eval(franka, moveRobot2,envData);

    //cout << "ExecuteAbility res = " << res->b << endl;

}

void callSkillExecuteFunction() {
    InstanceAccept<SkillConcept> skillGrasp("TestGraspSkill", {GraspSkill::getName()}, nlohmann::json{});
    InstanceAccept<FrankaRobotConcept> franka("FrankaPanda");
    Execution exec;

    franka.parameters->setPropertyValue<ConceptLibrary::EntityWithExecutorConcept::executorProperty::type>("executor", exec);
    InstanceAccept<GraspableObjectConcept> graspableObjInstance_from("MilkCartonLidlInstance1");  // "MilkCartonLidlInstance1" ,  "PlasticCupInstance2"
    InstanceAccept<GraspableObjectConcept> graspableObjInstance_into("BowlGreyIkeaInstance");
    InstanceAccept<GripperConcept> gripper = AndreiUtils::mapGet<String>(franka.parameters->getValue<AgentConcept::grippersProperty>().m, "FrankaPanda_FrankaGripper");
    InstanceAccept<EntityConcept> groundInstance("GroundInstance");
    gripper.parameters->setPropertyValue<ConceptLibrary::EntityWithExecutorConcept::executorProperty::type>("executor", exec);

    //skill.parameters->setPropertyValue<GraspSkill::fromLocationProperty::type>("location", Location{});

    EnvironmentData envData;
    // add the agent, gripper and object (and possibly others...) to the environmentData. Use AddAgentToEnvironment and AddObjectToEnvironment Functions
    AddAgentToEnvironment::eval(envData, InstanceAccept<AgentConcept>{franka});
    AddObjectToEnvironment::eval(envData, InstanceAccept<ObjectConcept>{graspableObjInstance_from} );
    AddObjectToEnvironment::eval(envData, InstanceAccept<ObjectConcept>{graspableObjInstance_into} );
    AddEntityToEnvironment::eval(envData, InstanceAccept<EntityConcept>{groundInstance} );


    exec.initializeEnvironmentFromSimulation(envData); // setting current locations
    // set skill properties: agent, gripper and object...
    auto graspEntitySetters = ConceptManager::getPropertySetters(GraspSkill::getName());
    AndreiUtils::mapGet<string>(graspEntitySetters, "a")(*skillGrasp.parameters, franka);
    AndreiUtils::mapGet<string>(graspEntitySetters, "o")(*skillGrasp.parameters, graspableObjInstance_from);
    AndreiUtils::mapGet<string>(graspEntitySetters, "g")(*skillGrasp.parameters, gripper);
    skillGrasp.parameters->setPropertyValue<GraspSkill::useSafeGraspSelectionPoseProperty::type>("useSafeGraspSelectionPose",trueBoolean);

    auto const GraspResult= skillGrasp.parameters->callFunction<ConceptLibrary::Boolean, EnvironmentData &, ConceptParameters &>("execution", envData, *skillGrasp.parameters);
    skillGrasp.parameters->callFunction<void, const EnvironmentData &, ConceptParameters &>("effects",envData, *skillGrasp.parameters); // right after execution, calling the effects of the function




    auto const goalForBowl= ConceptLibrary::Pose(AndreiUtils::Posed {Eigen::Quaterniond{0.7071,0,0,0.7071}, Eigen::Vector3d{0.4, 0.03, 0.0}});

    InstanceAccept<SkillConcept> skillTransport("TestTransportSkill", {TransportSkill::getName()}, nlohmann::json{});
    skillTransport.parameters->setPropertyValue<TransportSkill::toLocationProperty::type>("toLocation", Location{goalForBowl});
    auto transportEntitySetters = ConceptManager::getPropertySetters(TransportSkill::getName());
    AndreiUtils::mapGet<string>(transportEntitySetters, "a")(*skillTransport.parameters, franka);
    AndreiUtils::mapGet<string>(transportEntitySetters, "o")(*skillTransport.parameters, graspableObjInstance_from);
    AndreiUtils::mapGet<string>(transportEntitySetters, "g")(*skillTransport.parameters, gripper);

    //auto const TransportResult= skillTransport.parameters->callFunction<ConceptLibrary::Boolean, EnvironmentData &, ConceptParameters &>("execution", envData, *skillTransport.parameters);

    InstanceAccept<SkillConcept> skillPour("TestPourSkill", {PourSkill::getName()}, nlohmann::json{});
    skillPour.parameters->setPropertyValue<PourSkill::fromProperty::type>("from", graspableObjInstance_from);
    skillPour.parameters->setPropertyValue<PourSkill::intoProperty::type>("into", graspableObjInstance_into);
    skillPour.parameters->setPropertyValue<PourSkill::heightProperty::type>("height",Number(0.05));
    skillPour.parameters->setPropertyValue<PourSkill::angleProperty::type>("angle", Number(1.577));
    //skillPour.parameters->setPropertyValue<PourSkill::directionProperty::type>("direction",Number(0) );
    skillPour.parameters->setPropertyValue<PourSkill::timeProperty::type>("time",Number(1.5));
    skillPour.parameters->setPropertyValue<PourSkill::amountProperty::type>("amount", Number(0.15));
    skillPour.parameters->setPropertyValue<PourSkill::spinAngleProperty::type>("spinAngle", Number(-0.2));
    auto pourEntitySetters = ConceptManager::getPropertySetters(PourSkill::getName());
    AndreiUtils::mapGet<string>(pourEntitySetters, "a")(*skillPour.parameters, franka);
    AndreiUtils::mapGet<string>(pourEntitySetters, "g")(*skillPour.parameters, gripper);

    auto const pourResult= skillPour.parameters->callFunction<ConceptLibrary::Boolean, EnvironmentData &, ConceptParameters &>("execution", envData, *skillPour.parameters);
    skillPour.parameters->callFunction<void, EnvironmentData const &, ConceptParameters &>("effects",envData, *skillPour.parameters); // right after execution, calling the effects of the function


    auto const goalForRelease= ConceptLibrary::Pose(AndreiUtils::Posed {Eigen::Quaterniond{1, 0, 0, 0}, Eigen::Vector3d{0.4, 0.03, 0.0}});

    InstanceAccept<SkillConcept> skillRelease("TestReleaseSkill", {ReleaseSkill::getName()}, nlohmann::json{});
    skillRelease.parameters->setPropertyValue<ReleaseSkill::toLocationProperty::type>("toLocation", Location{goalForRelease});
    auto releaseEntitySetters = ConceptManager::getPropertySetters(ReleaseSkill::getName());
    AndreiUtils::mapGet<string>(releaseEntitySetters, "a")(*skillRelease.parameters, franka);
    AndreiUtils::mapGet<string>(releaseEntitySetters, "o")(*skillRelease.parameters, graspableObjInstance_from);
    AndreiUtils::mapGet<string>(releaseEntitySetters, "g")(*skillRelease.parameters, gripper);

    auto const releaseResult= skillRelease.parameters->callFunction<ConceptLibrary::Boolean, EnvironmentData &, ConceptParameters &>("execution", envData, *skillRelease.parameters);
    skillRelease.parameters->callFunction<void, const EnvironmentData &, ConceptParameters &>("effects",envData, *skillRelease.parameters); // right after execution, calling the effects of the function

    AndreiUtils::sleepMSec(3000);

}

int main() {
    cout << "Hello World!" << endl;

    auto config = readJsonFile("../config/testSimulatorArguments.json");

    ConceptLibrary::useAllGeneratedData();
    ConceptLibrary::LibraryInit::setConfigurationParameters(ConfigurationParameters(config.at("ConceptLibrary"), "ConceptLibrary"));
    ConceptLibrary::LibraryInit::init();
    RobotModelling::setConfigurationParameters(ConfigurationParameters(config.at("RobotModelling"), "RobotModelling"));



    //listObjectsInSimulation();
    //testMoveRobot();
    //testIKsolver();
    callSkillExecuteFunction();
    //testMotionPrimitiveExecution();
    //generateMotion();
    //useRobotToPour(config);

    return 0;
}

