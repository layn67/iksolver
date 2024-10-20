import { Object3D } from 'three';
import { IKChain } from './IKChain';
import { IKConfig } from './model';
import { CCDIKSolver } from './ccdIKSolver';

/**
 * Class responsible for solving inverse kinematics (IK) problems using an IK chain and a target.
 * The solver adjusts the IK chain to reach the target position, applying the CCD (Cyclic Coordinate Descent) algorithm.
 */
export class IKSolver {
    ikChain: IKChain | null = null;
    target : Object3D | null = null;

    private _config: IKConfig;
 
    constructor(config: Partial<IKConfig>) {
        this._config = {
            tolerance: 0.01,
            maxIterations: 10,
            updateURDFRobot: false,
            ...config,
        };

    }

    // Solves the IK problem by adjusting the IK chain to reach the target position
    solve() {
        if (!this.ikChain || !this.target) return;

        CCDIKSolver(this.ikChain, this.target.position, this._config);
        if (this._config.updateURDFRobot) this._updateURDFRobot();
    }

    // Updates the URDF robot model with the new joint positions
    private _updateURDFRobot() {
        if (!this.ikChain) return;
        for (const ikJoint of this.ikChain.ikJoints) {
            ikJoint.urdfJoint?.quaternion.copy(ikJoint.quaternion);
        }
    }
}
