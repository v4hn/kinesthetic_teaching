import pandas as pd
import scipy.interpolate as interpolate

def merge_trajectories(t):
    '''
    @param t: list of pandas DataFrames, each with a column 'time_from_start' and columns for each joint
    @return: a single DataFrame with column 'time_from_start' and columns for each joint
    '''
    all_joints = pd.DataFrame(t[0])
    for tn in t[1:]:
        for j in tn.columns[1:]: # skip time_from_start
            fn = interpolate.interp1d(tn['time_from_start'], tn[j], fill_value='extrapolate')
            all_joints[j] = fn(all_joints['time_from_start'])
    return all_joints

def fn_from_trajectory(trajectory : pd.DataFrame):
    """
    @param trajectory: pandas DataFrame with columns 'time_from_start' and joint columns
    @returns map of function approximators indexed by joint names
    """

    fns = { j : interpolate.interp1d(trajectory['time_from_start'], trajectory[j], fill_value='extrapolate') for j in trajectory.columns[1:]}
    return 