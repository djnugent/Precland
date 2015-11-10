import numpy as np


def Cart2Ind(offset,x,top):
    return np.clip(np.round(offset+x),0,top).astype(int)

class EllipseScanner():
    def __init__(self, r_min=0.6, r_max=0.8, n_ellipses=4, n_scan_points=360):
        # initialize set of concentric ellipses used to scan a region to determine if a target is present
        self.ellipses = self.init_scan_ellipses(r_min, r_max, n_ellipses, n_scan_points)

    def init_scan_ellipses(self, r_min, r_max, n_ellipses, n_scan_points):
        # Define scanning ellipses (currently circles, to be more specific) in spherical coords
        radii = np.linspace(r_min, r_max, num=n_ellipses, endpoint=True)
        thetas_rad = np.linspace(0,2*np.pi,num=n_scan_points,endpoint=False)

        # Convert from spherical to Cartesian
        x = np.outer(radii/2.0,np.cos(thetas_rad))
        y = np.outer(radii/2.0,np.sin(thetas_rad))

        ellipses = []
        for i in xrange(n_ellipses):
            ellipses.append(np.array([x[i,:],y[i,:]]))

        return ellipses

    def get_affine_transform(self, h, w, theta):
        M_rot = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
        M_scale = np.array([[w, 0],[0, h]])
        A = M_rot.dot(M_scale)
        return A

    def scan_img(self, img, h, w, theta):
        h,w  = img.shape
        if not self.ellipses:
            print ("[scan_img] Error - Scanning ellipses must be initialized before attempting to scan.")
            return None

        # Define affine transform used to transform scanning circles into ellipses
        A = self.get_affine_transform(h, w, theta)

        # Transform scanning ellipses to match img
        ellipses_affine = []
        for e in self.ellipses:
            ellipses_affine.append(np.dot(A,e))

        # Find image indices pertaining to Cartesian scanning points
        ellipses_img_ind = []
        for e in ellipses_affine:
            ellipses_img_ind.append(np.array([Cart2Ind(e[0,:],h/2.0,h-1),Cart2Ind(e[1,:],w/2.0,w-1)]))

        # Extract image data and form 1D arrays for each ellipse scan
        scan_result = []
        for e in ellipses_img_ind:
            scan_result.append(img[e[0,:],e[1,:]])


        return np.round(np.mean(scan_result,axis=0)/255.0).astype(int), ellipses_img_ind
