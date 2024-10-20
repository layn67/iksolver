import { Group, Object3D, Vector3 } from "three";
import { URDFJoint, URDFRobot } from "urdf-loader";
import { AxisName, JointLimit } from "./model";

// Default axis direction used if none is specified in URDFJoint
const DEFAULT_AXIS = new Vector3(0, 1 , 0);

// Represents a joint in an inverse kinematics (IK) chain
/**
 * `IKJoint` extends `Object3D` to represent a joint in an IK chain.
 * It incorporates optional URDF joint data for initialization.
 * 
 * **Key Properties**:
 * - `isRootJoint`: Determines if this joint is the root of the chain.
 * - `axis`: The axis of rotation or movement for this joint, defaulting to `DEFAULT_AXIS`.
 * - `axisName`: The name of the axis, if defined.
 * - `limit`: Joint limits, if specified.
 * - `isHinge`: Whether the joint is a revolute (hinge) type.
 * - `isFixed`: Whether the joint is a fixed type.
 */
export class IKJoint extends Object3D {

    constructor(private _urdfJoint?: URDFJoint) {
        super();
        if (this.urdfJoint) {
            this.position.copy(this.urdfJoint.position);
            this.rotation.copy(this.urdfJoint.rotation);
        }
    }

    get isRootJoint(): boolean {
        return !this._urdfJoint;
    }

    get urdfJoint() {
        return this._urdfJoint;
    }

    get axis() {
        return this._urdfJoint?.axis || DEFAULT_AXIS;
    }

    get axisName(): AxisName | '' {
        return Object.entries(this.axis).find(([key, value]) => value)?.[0] as AxisName || '';
    }

    get limit(): JointLimit {
        return this._urdfJoint?.limit as JointLimit || {lower: 0, upper: 0};
    }

    get isHinge(): boolean {
        return this._urdfJoint?.jointType === 'revolute';
    }

    get isFixed(): boolean {
        return this._urdfJoint?.jointType === 'fixed';
    }
}

// Represents a chain of IKJoints forming a kinematic chain
/**
 * `IKChain` manages a sequence of `IKJoint` objects, forming a kinematic chain.
 * It can be constructed from a URDF robot model and includes methods for 
 * traversing and building the joint hierarchy.
 * 
 * **Key Properties**:
 * - `endEffector`: The final joint in the chain.
 * - `ikJoints`: All joints in the chain, including the root joint.
 * 
 * **Key Methods**:
 * - `createFromURDF(robot: URDFRobot)`: Builds the IK chain from a URDF robot and returns a `Group` containing both the robot and the IK chain.
 */
export class IKChain {
    private _rootJoint: IKJoint;
    private _endEffector!: IKJoint;
    private _ikJoints: IKJoint[] | undefined = undefined;

    constructor() {
        // Initialize with a root joint
        this._rootJoint = new IKJoint();
    }

    get endEffector() {
        return this._endEffector;
    }

    // Retrieves all IKJoints in the chain, including the root joint
    get ikJoints(): IKJoint[] {
        if (!this._ikJoints) {
            this._ikJoints = [];
            this._rootJoint.traverse((child) => child instanceof IKJoint && this.ikJoints.push(child as IKJoint));
        }
        return this._ikJoints;
    }

    // Creates an IK chain from a URDF robot and returns a Group containing both
    createFromURDF(robot: URDFRobot): Group {
        this._traverseURDFJoints(
            this._rootJoint,
            this._baseURDFJoint(robot)
        );

        const robotGroup = new Group();
        robotGroup.add(robot, this._rootJoint);

        return robotGroup;
    }

    // Recursively traverses URDF joints and builds the IKJoint hierarchy
    private _traverseURDFJoints(parentIKJoint: IKJoint, urdfJoint: URDFJoint) {
        const ikJoint = new IKJoint(urdfJoint);
        parentIKJoint.add(ikJoint);
        parentIKJoint = ikJoint;

        const nextUrdfJoint = this._nextURDFJoint(urdfJoint);

        if (!nextUrdfJoint) {
            this._endEffector = ikJoint;
            return;
        }
        this._traverseURDFJoints(parentIKJoint, nextUrdfJoint!);
    }

    // Finds the base URDF joint from the children of the given parent Object3D
    private _baseURDFJoint(parent: Object3D) {
        return parent.children
            .find((child) => (child as URDFJoint).isURDFJoint && this._nextURDFJoint(child)) as URDFJoint;
    }

    // Finds the next URDF joint in the hierarchy
    private _nextURDFJoint(parent: Object3D) {
        return parent.children[0].children.find((child) => (child as URDFJoint).isURDFJoint) as URDFJoint;
    }
}
