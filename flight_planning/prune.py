import numpy as np
def collinear_points(p1, p2, p3):
         
        collinear = False
        det = p1[0]*(p2[1] - p3[1]) + p2[0]*(p3[1] - p1[1]) + p3[0]*(p1[1] - p2[1])
        if det == 0 :
            collinear = True
    
        return collinear
    
def path_prune(path):
    
        pruned_path = [p for p in path]
    
        i = 0
        while i < len(pruned_path) - 2:
            p1 = pruned_path[i]
            p2 = pruned_path[i+1]
            p3 = pruned_path[i+2]
            
            if collinear_points(p1, p2, p3):
                pruned_path.remove(pruned_path[i+1])
            else:
                i += 1
        return pruned_path   
    
#def collinear_points(p1, p2, p3):
#        """
#        Check collinearity by using vector cross product
#        """
#        return np.allclose(np.cross(np.array(p1) - np.array(p2), np.array(p2) - np.array(p3)), (0, 0, 0))


#def path_prune(path, collinear_fn):
#        """
#        prune the path, i.e. remove unnecessary waypoints that are collinear to each other
#        """
#        if len(path) <= 1:
#            return path[:]
#        new_path = [path[0]]
#        line_start = path[0]
#        line_end = path[1]
#        for i in range(2, len(path)):
#            next_end = path[i]
#            if collinear_fn(line_start, line_end, next_end):
#                line_end = next_end
#            else:
#                new_path.append(line_end)
#                line_start = line_end
#                line_end = next_end
#        new_path.append(line_end)
#        return new_path
