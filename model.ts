export type AxisName = 'x' | 'y' | 'z';

export interface IKConfig {
    tolerance: number,
    maxIterations: number,
    updateURDFRobot: boolean,
}

export interface JointLimit {
    lower: number,
    upper: number,
}

export interface LinkHelperConfig {
    linkColor: number,
    linkRadialSegments: number,
    linkTubularSegments: number,
    linkRadius: number,
}

export interface JointHelperConfig {
    jointColor: number,
    jointRadialSegments: number,
    jointRadius: number,
    jointHeight: number,
}

export interface IKHelperConfig extends LinkHelperConfig, JointHelperConfig {}
