#!/usr/bin/env python

import math 
import numpy as np
from PIL import Image

class AStarAlgorithm:
    def __init__(self, image_path):

        # Load the BMP file using PIL
        image = Image.open(image_path)  # Replace 'your_image.bmp' with the path to your BMP file

        # Convert the image to a NumPy array
        brushfire_grid = np.array(image)

        # Create a copy of the input array to avoid modifying the original array
        self.modified_array = np.copy(brushfire_grid)

        # Replace values greater than 1 with 1
        self.modified_array[brushfire_grid < 70] = 1
        self.modified_array[brushfire_grid >= 70] = 0

    def m_cost(self, node_a, node_b):
        # Pour une carte, le coût est toujours de 1.
        # On s'assure tout de même que les cellules sont adjacentes.
        dist_x = abs(node_a[0] - node_b[0])
        dist_y = abs(node_a[1] - node_b[1])
        assert dist_x <= 1 and dist_y <= 1, "m_cost : %s is not a neighbor of %s"%(str(node_a), str(node_b))
        return 1

    def m_neighbors_8(self, node, obs_map):
        # Génère les voisins valides de node selon la carte (obs_map)
        # Connectivité 8.
        # Retourne une liste de tuples dont les cases sont vides (=0).
        ns = []
        x = node[0]
        y = node[1]
        lx = obs_map.shape[0] - 1
        ly = obs_map.shape[1] - 1

        min_x = -1 if (x > 0) else 0
        min_y = -1 if (y > 0) else 0
        max_x = 2 if (x < lx) else 1
        max_y = 2 if (y < ly) else 1

        for dx in range(min_x, max_x):
            for dy in range(min_y, max_y):
                if ((dx == 0) and (dy == 0)):
                    continue
                n = (x+dx, y+dy)
                if (obs_map[n] == 0):
                        ns.append(n)
            
        return ns

    def m_h(self, node_a, node_b):

        (ax, ay) = node_a
        (bx, by) = node_b
        return math.sqrt((ax-bx)**2 + (ay-by)**2)


    def astar(self, start, goal, c_fun, n_fun, h_fun):
        # Cherche le chemin le plus court entre start et goal à l'aide de l'algorithme A*.
        # Retourne un tuple de la séquence et du coût : (seq, cost)
        # Utilise les fonctions :
        #   c_fun(edge): retourne le coût de parcours du lien edge
        #   n_fun(node): retourne les voisins du noeud node
        #   h_fun(node_a, node_b): retourne la valeur de l'heuristique du coût de parcours entre node_a et node_b

        from math import inf # Valeur infinie

        search_set = [start] # Ensemble de recherche, ne contient que le noeud de départ pour l'instant.

        # Dictionnaire contenant le plus bas coût réel du chemin (G) jusqu'au noeud spécifié.
        g = {}
        # On initialise avec g = 0 pour le départ
        g[start] = 0

        # Dictionnaire contenant la plus basse valeur de la fonction f(node) pour chaque noeud.
        f = {} 
        # On initialise f(start) avec g (qui est 0) et l'heuristique jusqu'à la fin.
        f[start] = g[start] + h_fun(start, goal)

        # Dictionnaire qui conserve pour chaque noeud la provenance permettant la valeur f la plus faible.
        # Utilisé pour reconstruire le chemin lorsque l'objectif est atteint.
        from_node = {}
        # On initialise avec le noeud de départ sans provenance (None).
        from_node[start] = None

        while (len(search_set) > 0):
            # On trouve le noeud ayant la plus petite valeur f dans l'ensemble de recherche :
            min_f = inf
            min_n = None
            for n in search_set:
                assert n in f, "Error : %s not in F!"%(str(n))
                if f[n] < min_f:
                    min_f = f[n]
                    min_n = n
            
            # On retire le noeud ayant le plus petit 'f' et on poursuit avec lui :
            current = min_n
            print("Current: %s, F(%s) = %d"%(current, current, min_f))
            search_set.remove(min_n)
            
            # Si le noeud en cours est l'objectif, c'est qu'aucun autre noeud dans l'espace de recherche n'a
            # un meilleur potentiel du chemin le plus court. On reconstruit le chemin ensuite. 
            if (current == goal):
                path_r = [goal]
                previous = from_node[goal]
                while (previous is not None):
                    path_r.append(previous)
                    previous = from_node[previous]             
                path_r.reverse() # On remet la liste dans le bon ordre
                return (path_r, g[goal])
            
            ns = n_fun(current) # Les voisins du noeud en cours
            for n in ns:
                # Pour chaque voisin, on calcule sa fonction f et on l'ajoute à l'ensemble de recherche seulement si
                # la valeur de g est plus basse que celle déjà connue pour ce noeud
                g_n = g[current] + c_fun(current, n)
                f_n = g_n + h_fun(n, goal)
                print("G(%s) = %d, F(%s) = %d"%(n, g_n, n, f_n))
                if ((n not in g) or (g_n < g[n])):
                    from_node[n] = current
                    g[n] = g_n
                    f[n] = f_n
                    if (n not in search_set):
                        print("Adding %s"%(str(n)))
                        search_set.append(n)

        # Nous avons vidé l'espace de recherche sans trouver de solution. Retourner une liste vide et un coût négatif.
        return ([], -1)

    def m_n(self, node):
        x, y = node
        neighbors = []

        for dx in range(-1, 2):
            for dy in range(-1, 2):
                if dx == 0 and dy == 0:
                    continue

                nx, ny = x + dx, y + dy
                if 0 <= nx < self.modified_array.shape[0] and 0 <= ny < self.modified_array.shape[1]:
                    if self.modified_array[nx, ny] == 0:
                        neighbors.append((nx, ny))

        return neighbors
    
#def main():
   #m_start = (315,167)
   #m_goal = (59,135)
   #m_n = lambda node : m_neighbors_8(node, modified_array)
   #(seq, cost) = astar(m_start, m_goal, m_cost, m_n, m_h)
                       
   #print("Le plus court chemin entre %s et %s est : %s (%d)"%(m_start, m_goal, seq, cost))

#if __name__ == '__main__':
 #main()
    