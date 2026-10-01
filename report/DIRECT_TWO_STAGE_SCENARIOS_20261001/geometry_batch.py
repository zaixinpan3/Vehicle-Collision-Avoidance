"""Vectorized rectangle distances and strict SAT overlap for saved replay states."""
import numpy as np


def sampled_geometry(trace, target_initial, ego_shape):
    states=np.asarray([x for hold in trace for x in hold['auditStates']],dtype=float)
    times=np.asarray([hold['time']+dt for hold in trace for dt in hold['auditTimes']])
    q=np.asarray(target_initial,dtype=float)
    arc=q[3]*times+.5*q[4]*times**2
    curvature=np.sin(q[5])/q[6]
    angle=curvature*arc/2
    course=q[2]+q[5]+angle
    travel=arc*np.sinc(angle/np.pi)
    target=np.column_stack((q[0]+travel*np.cos(course),q[1]+travel*np.sin(course),q[2]+curvature*arc))
    ego,ea=vertices(states[:,:3],ego_shape)
    obstacle,ta=vertices(target,q[7:11])
    axes=np.concatenate((ea,ta),axis=1)
    pe=np.einsum('nki,nai->nka',ego,axes)
    pt=np.einsum('nki,nai->nka',obstacle,axes)
    gaps=np.maximum(pe.min(axis=1)-pt.max(axis=1),pt.min(axis=1)-pe.max(axis=1)).max(axis=1)
    distances=np.sqrt(np.minimum(vertex_edge_distance_squared(ego,obstacle),vertex_edge_distance_squared(obstacle,ego)))
    distances[gaps<=0]=0
    return times,states,target,distances,np.maximum(0,-gaps)


def vertices(pose,shape):
    x=np.column_stack((np.cos(pose[:,2]),np.sin(pose[:,2])))
    y=np.column_stack((-x[:,1],x[:,0]))
    center=pose[:,:2]+shape[2]*x+shape[3]*y
    sx=np.array([-1,1,1,-1]);sy=np.array([-1,-1,1,1])
    return center[:,None,:]+shape[0]*sx[None,:,None]*x[:,None,:]+shape[1]*sy[None,:,None]*y[:,None,:],np.stack((x,y),axis=1)


def vertex_edge_distance_squared(points,polygon):
    edges=np.roll(polygon,-1,axis=1)-polygon
    delta=points[:,:,None,:]-polygon[:,None,:,:]
    fraction=np.clip(np.sum(delta*edges[:,None,:,:],axis=-1)/np.sum(edges**2,axis=-1)[:,None,:],0,1)
    offset=delta-fraction[:,:,:,None]*edges[:,None,:,:]
    return np.sum(offset**2,axis=-1).min(axis=(1,2))
