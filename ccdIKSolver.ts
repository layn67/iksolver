import { Quaternion, Vector3 } from 'three';
import { IKChain, IKJoint } from './IKChain';
import { IKConfig, JointLimit, AxisName } from './model';

/**
 * `CCDIKSolver` iteratively adjusts the joints of an `IKChain` to minimize the distance 
 * between the end effector and a target position using the Cyclic Coordinate Descent (CCD) method.
 * 
 * **Parameters**:
 * - `ikChain`: The kinematic chain of joints to adjust.
 * - `targetPos`: The target position in world coordinates for the end effector.
 * - `config`: Configuration settings including tolerance and maximum iterations.
 */
export function CCDIKSolver(ikChain: IKChain, targetPos: Vector3, config: IKConfig) {
    const { ikJoints, endEffector } = ikChain;

    let iteration = 0;
    // Calculate the distance from the end effector to the target position
    const calcEndEffectorTargetDistance = () => endEffector.worldToLocal(targetPos.clone()).length();
    let endEffectorTargetDistance = calcEndEffectorTargetDistance();
    
    // Temporary vectors and quaternions for calculations
    const endEffectorWorldPos = new Vector3();
    const jointToTargetQt = new Quaternion();
    const inverseQt = new Quaternion();
    const jointAxisInverseQt = new Vector3();

    // Iteratively adjust joint orientations to minimize the distance to the target
    while (endEffectorTargetDistance > config.tolerance && iteration <= config.maxIterations) {
        for (let i = ikJoints.length - 2; i >= 0; i--) {
            const ikJoint = ikJoints[i];

            // Only adjust joints that are not fixed
            if (!ikJoint.isFixed) {
                endEffector.getWorldPosition(endEffectorWorldPos);
                const directionToEndEffector = ikJoint.worldToLocal(endEffectorWorldPos.clone()).normalize();
                const directionToTarget = ikJoint.worldToLocal(targetPos.clone()).normalize();
                jointToTargetQt.setFromUnitVectors(directionToEndEffector, directionToTarget);
                ikJoint.quaternion.multiply(jointToTargetQt);

                if (ikJoint.isHinge || ikJoint.isRootJoint) {
                    inverseQt.copy(ikJoint.quaternion).invert();
                    jointAxisInverseQt.copy(ikJoint.axis).applyQuaternion(inverseQt);
                    jointToTargetQt.setFromUnitVectors(ikJoint.axis, jointAxisInverseQt);
                    ikJoint.quaternion.multiply(jointToTargetQt);
                }

                // Clamp the joint rotation to its limits if needed
                if (ikJoint.limit) {
                    const ikJointRotationAngle = getIKJointRotationAngle(ikJoint);
                    const [clampedRotationAngle, isClamped] = clampIKJointRotationAngle(ikJointRotationAngle, ikJoint.limit);
                    if (isClamped)
                        ikJoint.quaternion.setFromAxisAngle(ikJoint.axis, clampedRotationAngle)
                }
            }

            ikJoint.updateMatrixWorld();
        }
        endEffectorTargetDistance = calcEndEffectorTargetDistance();
        iteration++;
    }

}

/**
 * Calculates the rotation angle of an `IKJoint` based on its quaternion and rotation axis.
 * 
 * **Parameters**:
 * - `ikJoint`: The joint for which to calculate the rotation angle.
 * 
 * **Returns**: The rotation angle in radians.
 */
function getIKJointRotationAngle(ikJoint: IKJoint): number {
    const { axisName, axis } = ikJoint;
    return Math.asin(ikJoint.quaternion[axisName as AxisName] / axis[axisName as AxisName]) * 2;
}

/**
 * Clamps the joint rotation angle within its specified limits.
 * 
 * **Parameters**:
 * - `ikJointRotationAngle`: The angle to clamp.
 * - `limit`: The joint's rotation limits.
 * 
 * **Returns**: A tuple containing the clamped angle and a boolean indicating if clamping occurred.
 */
function clampIKJointRotationAngle(ikJointRotationAngle: number, limit: JointLimit): [number, boolean] {
    const { lower, upper } = limit;
    let isClamped = false;
    if (ikJointRotationAngle < lower) {
        isClamped = true;
        ikJointRotationAngle = lower;
    }

    else if (ikJointRotationAngle > upper) {
        isClamped = true;
        ikJointRotationAngle = upper;
    }

    return [ikJointRotationAngle, isClamped];
}

