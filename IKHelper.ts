import { AxesHelper, CatmullRomCurve3, CylinderGeometry, Mesh, MeshBasicMaterial, TubeGeometry, Vector3 } from 'three';
import { IKChain } from './IKChain';
import { JointHelperConfig, LinkHelperConfig, IKHelperConfig } from './model';

const COLOR = 0x0092ff;
const SEGMENTS = 8;

// Default configuration for link visualization
const LINK_CONFIG: LinkHelperConfig = {
    linkColor: COLOR,
    linkRadialSegments: SEGMENTS,
    linkTubularSegments: 64,
    linkRadius: 0.001,
}

// Default configuration for joint visualization
const JOINT_CONFIG: JointHelperConfig = {
    jointColor: COLOR,
    jointRadialSegments: SEGMENTS,
    jointRadius: 0.01,
    jointHeight: 0.05,
}

// Helper class for visualizing an IK chain
/**
 * `IKHelper` visualizes an Inverse Kinematics (IK) chain using Three.js.
 * It generates 3D models for joints and links, helping to visually debug and understand
 * the IK configuration. Joints are represented as cylinders, and links as tubes connecting
 * these joints.
 * 
 * **Constructor Parameters**:
 * - `_ikChain`: The IK chain to visualize.
 * - `config`: Optional configuration to customize appearance.
 * 
 * **Key Methods**:
 * - `visualizeIKChain()`: Creates and positions visual representations for the IK chain's joints and links.
 */
export class IKHelper {
    private _config: IKHelperConfig;

    constructor(
        private _ikChain: IKChain,
        config: Partial<IKHelperConfig> = {}
    ) {
        this._config = {
            ...LINK_CONFIG,
            ...JOINT_CONFIG,
            ...config
        }
    }

    // Visualizes the IK chain by creating visual representations of joints and links
    visualizeIKChain() {
        const ikJoints = this._ikChain.ikJoints;
        const joint = this._createJoint();
        const linkMaterial = this._createLinkMaterial();

        let parent = ikJoints[0].parent;

        for (let i = 0; i < ikJoints.length; i++) {
            const ikJoint = ikJoints[i];
            const linkGeometry = this._createLinkGeometry(ikJoint.position);
            const link = new Mesh(linkGeometry, linkMaterial);
            parent?.add(link);
            parent = ikJoint;

            if (!i || i === ikJoints.length - 1) continue;

            if (ikJoint.axisName === 'z')
                joint.rotateX(Math.PI / 2);
            const axesHelper = new AxesHelper(0.1);
            ikJoint.add(axesHelper, joint);
        }

    }

    // Creates the geometry for a link between two joints
    private _createLinkGeometry(endPoint: Vector3) {
        const startPoint = new Vector3(0, 0, 0);
        const linkPath = new CatmullRomCurve3([startPoint, endPoint]);

        return new TubeGeometry(
            linkPath,
            this._config.linkTubularSegments,
            this._config.linkRadius,
            this._config.linkRadialSegments
        );
    }

    private _createLinkMaterial() {
        return new MeshBasicMaterial({
            color: this._config.linkColor,
        });

    }

    // Creates the geometry for a joint. Create and return a Mesh for the joint
    private _createJoint() {
        const jointGeometry = new CylinderGeometry(
            this._config.jointRadius,
            this._config.jointRadius,
            this._config.jointHeight,
            this._config.jointRadialSegments
        );
        const jointMaterial = new MeshBasicMaterial({
            color: this._config.jointColor
        });

        return new Mesh(jointGeometry, jointMaterial);
    }
}