def setup():
    size(800,450)
    noStroke()
    
    global nc, nr, states
    global dm, pth
    global dx,dy
    # number of columns and number of rows
    nc,nr = 48, 27
    # cellsize in x and y
    dx = width/float(nc)
    dy = height/float(nr)
    pth = None
    
    # initial configuration with a valid solution
    while pth is None:
        reset()
    
def draw():
    # display cells
    noStroke()
    for r in range(nr):
        for c in range(nc):
            fill(200+states[r][c]*55)
            rect(c*dx,r*dy,dx,dy)
    
    # display path
    if pth is not None:
        stroke(255,0,127)
        strokeWeight(4)
        noFill()
        beginShape()
        for p in pth:
            vertex(dx/2+p[1]*dx,dy/2+p[2]*dy)
        endShape()
    
    # run 50 cycles of evolution
    for i in range(50):
        evolve()

def get_fitness():
    if pth is not None:
        return len(pth)

def evolve():
    f0 = get_fitness()
    print 'fitness:',f0
    i1 = int(random(nc*nr))
    i2 = int(random(nc*nr))
    
    global states, dm, pth
    x1,y1 = get_coords(i1)
    x2,y2 = get_coords(i2)
    
    pt = [p for p in pth]
    
    # swap two random cells
    t = states[y1][x1]
    states[y1][x1] = states[y2][x2]
    states[y2][x2] = t
    
    # calculate shortest path
    dm,pth = dijkstra(states,(1,1),(nc-2,nr-2))
    
    # if no path is found, swap back
    if pth is None:
        t = states[y1][x1]
        states[y1][x1] = states[y2][x2]
        states[y2][x2] = t
        pth = [p for p in pt]
        return
    
    # else calculate new fitness
    f1 = get_fitness()
    
    # if no improvement, swap back
    if f0>f1:
        t = states[y1][x1]
        states[y1][x1] = states[y2][x2]
        states[y2][x2] = t
        pth = [p for p in pt]

def get_index(x,y):
    return y*nc+x

def get_coords(i):
    x = i%nc
    y = i/nc
    return x,y

# initialize grid and compute shortest path
def reset():
    global states, dm, pth
    states = [[int(random(1)>0.4) for x in range(nc)] for y in range(nr)]
    states[1][1] = 1
    dm,pth = dijkstra(states,(1,1),(nc-2,nr-2))

# Dijkstra's shortest path algorithm
def dijkstra(occ_map, start, goal):
    # https://en.wikipedia.org/wiki/Von_Neumann_neighborhood
    directions = [(1,0),(-1,0),(0,1),(0,-1)]
    goal_found = False

    possible_nodes = [[ 0 for x in range(nc)] for y in range(nr)]
    dist_map =       [[-1 for x in range(nc)] for y in range(nr)]

    current_x, current_y = start
    possible_nodes[current_y][current_x] = 5 # somewhat arbitrary number, good for plotting with matplotlib
    dist_map[current_y][current_x] = 0

    g_value = 0
    frontier_nodes = [(g_value, current_x, current_y)] # dist, x, y
    # searched_nodes = []
    parent_node = {}  # Dictionary that Maps {child node : parent node}

    while len(frontier_nodes) != 0:
        frontier_nodes.sort(reverse=True) #sort from shortest distance to farthest
        current_node = frontier_nodes.pop()
        if current_node[1] == goal[0] and current_node[2] == goal[1]:
            # print "Goal found!"
            goal_found = True
            break
        g_value, current_x, current_y = current_node
        dist_map[current_y][current_x] = g_value

        for di in directions:
            possible_expansion_x = current_x + di[0]
            possible_expansion_y = current_y + di[1]

            if 0<=possible_expansion_x<nc and 0<=possible_expansion_y<nr:
                try:
                    unsearched_node = possible_nodes[possible_expansion_y][possible_expansion_x] == 0
                    open_node = occ_map[possible_expansion_y][possible_expansion_x] == 1
                except:
                    unsearched_node = False
                    open_node = False
                if unsearched_node and open_node:
                    possible_nodes[possible_expansion_y][possible_expansion_x] = 3
                    cost = 1
                    possible_node = (g_value + cost, possible_expansion_x, possible_expansion_y)
                    frontier_nodes.append(possible_node)
                    parent_node[possible_node] = current_node

    if goal_found:
        # print "Generating path..."
        route = []
        route.append((g_value,goal[0],goal[1]))
        child_node = current_node
        while parent_node.has_key(child_node):
            route.append(parent_node[child_node])
            child_node = parent_node[child_node]
            #route.sort()
        route.reverse()

        return dist_map, route

    else:
        #print "goal not found"
        return dist_map, None
