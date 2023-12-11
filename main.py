import utm
from math import sqrt
from polycircles import polycircles
import simplekml
import math
from midpoint import midpoint

show_animation = True

obx = []
oby = []
radius = []
obstacles_2 = []
obxint = []
obyint = []
radiusint = []

def read_obstacle(obstacle_file):
    obstacle_ = []
    ob = []
    with open(obstacle_file) as of:
        line = of.readline()
        obstacle_.append(line.split(" "))
        while line:
            line = of.readline()
            obstacle_.append(line.split(" "))
    ob = obstacle_[2:]
    for i in range(len(ob)):
        ob[i][0] = ob[i][0].split('\t')
    ob = ob[:-1]
    for h in range(len(ob)):
        lati = ob[h][0][8]
        longi = ob[h][0][9]
        rad = ob[h][0][10]

        obstacles_2.append([float(lati), float(longi)])
        obx.append(float(lati))
        oby.append(float(longi))
        radius.append(float(rad))

inter_wp_x = []
inter_wp_y = []
inter_wp = []

final_inter_wp = []

def inter_waypoint(inter_waypoint):
    waypoint_ = []
    wp = []
    with open(inter_waypoint) as of:
        line = of.readline()
        waypoint_.append(line.split(" "))
        while line:
            line = of.readline()
            waypoint_.append(line.split(" "))
    wp = waypoint_[2:]
    for a in range(len(wp)):
        wp[a][0] = wp[a][0].split('\t')
    wp = wp[:-1]
    for c in range(len(wp)):
        lati = wp[c][0][8]
        longi = wp[c][0][9]

        inter_wp_x.append(float(lati))
        inter_wp_y.append(float(longi))
        inter_wp.append([float(lati), float(longi)])

obstacle_file = r"./input/obstacles.waypoints"
read_obstacle(obstacle_file)

def ftoi(float_array, int_array):
    for i in range(len(float_array)):
        int_array.append(math.floor(float_array[i]))

ftoi(radius, radiusint)

safety_distance = int(input("\nEnter the Safety Distance you want Beyond the Radius of Circle (in Feet): "))
saf_radius = []

for b in range(len(radius)):
    radius[b] = float(radius[b]) + safety_distance

class Node:
    def __init__(self, x, y, cost, pind):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)


def calc_final_path(ngoal, closedset, reso):
    rx, ry = [ngoal.x * reso], [ngoal.y * reso]
    pind = ngoal.pind
    while pind != -1:
        n = closedset[pind]
        rx.append(n.x * reso)
        ry.append(n.y * reso)
        pind = n.pind

    return rx, ry

def a_star_planning(sx, sy, gx, gy, minx, miny, maxx, maxy, reso, rr):
    nstart = Node((sx / reso), (sy / reso), 0.0, -1)
    ngoal = Node((gx / reso), (gy / reso), 0.0, -1)
    motion = get_motion_model()

    openset, closedset = dict(), dict()
    openset[calc_index(nstart, minx, minx, miny)] = nstart

    while 1:
        if openset == {}:
            break
        c_id = min(openset, key=lambda o: openset[o].cost + calc_heuristic(ngoal, openset[o]))
        current = openset[c_id]

        delta = 1
        if current.x >= ngoal.x - delta and current.x <= ngoal.x + delta and current.y >= ngoal.y - delta and current.y <= ngoal.y + delta:
            ngoal.pind = current.pind
            ngoal.cost = current.cost
            break

        del openset[c_id]
        closedset[c_id] = current

        for i, _ in enumerate(motion):
            node = Node(current.x + motion[i][0],
                                    current.y + motion[i][1],
                                    current.cost + motion[i][2], c_id)
            n_id = calc_index(node, minx, minx, miny)

            if n_id in closedset:
                continue

            if not verify_node(node, minx, miny, maxx, maxy):
                continue

            if n_id not in openset:
                openset[n_id] = node
            else:
                if openset[n_id].cost >= node.cost:
                    openset[n_id] = node

    rx, ry = calc_final_path(ngoal, closedset, reso)

    return rx, ry

def calc_heuristic(n1, n2):
    w = 1.0
    d = w * sqrt((n1.x - n2.x)**2 + (n1.y - n2.y)**2)
    return d

def is_obstacle(node):
    for cx, cy, r in zip(obx, oby, radius):
        d1 = sqrt((cx - node.x)**2 + (cy - node.y)**2) - 0.5
        if d1 <= r/1.0:
            return True
    return False


def verify_node(node, minx, miny, maxx, maxy):

    if node.x < minx:
        return False
    elif node.y < miny:
        return False
    elif node.x >= maxx:
        return False
    elif node.y >= maxy:
        return False
    elif is_obstacle(node):
        return False

    return True

def calc_index(node, xwidth, xmin, ymin):
    return (node.y - ymin) * xwidth + (node.x - xmin)

def get_motion_model():
    motion = [[1, 0, 1],
                      [0, 1, 1],
                      [-1, 0, 1],
                      [0, -1, 1],
                      [-1, -1, sqrt(2)],
                      [-1, 1, sqrt(2)],
                      [1, -1, sqrt(2)],
                      [1, 1, sqrt(2)]]

    return motion

def inter():
    grid_size = 1.0
    drone_size = 1.0
    
    input_wp_file_path = r"./input/wp.waypoints"
    inputfile = open(input_wp_file_path, "r")
    
    inter_wp_file_path = r"./inter/inter.waypoints"
    outputfile = open(inter_wp_file_path, "w")

    line = inputfile.readline()
    outputfile.write(line)
    line = inputfile.readline()

    waypoints = []
    altitude = []
    utmCo = []
    lines = []
    while line:
        if (len(line) > 0):
            lines.append(line)
            line = line.split()
            lx = float(line[8])
            ly = float(line[9])
            alt = float(line[10])
            waypoints.append((lx, ly))
            altitude.append(alt)
        line = inputfile.readline()

    length = len(waypoints)
    outputfile.write(lines[0])
    
    # converting the initial waypoints to utm coordinates
    for d in range(length):
        coordinates = utm.from_latlon(waypoints[d][0], waypoints[d][1])
        utmCo.append(coordinates)

    # No. of waypoints in the final output file
    Count = 0

    # converting the obstacle waypoints to utm coordinates
    for e in range(len(obx)):
        coordinates = utm.from_latlon(obx[e], oby[e])
        obx[e] = coordinates[0]
        oby[e] = coordinates[1]

    for f in range(length-1):
        j = f+1
        # initial waypoint and final waypoint
        sx = utmCo[f][0]
        sy = utmCo[f][1]
        gx = utmCo[j][0]
        gy = utmCo[j][1]

        prev_x = sx
        prev_y = sy
        prev_wp_is_obstacle = False

        current_x = sx+1
        m = (gy - sy) / (gx - sx)
        c = sy - (m * sx)

        miniwp = []

        if (current_x < gx):
            while current_x < gx:
                current_y = m*current_x + c
                node = Node((current_x/grid_size), (current_y/grid_size), 0, 0)
                cur_wp_is_obstacle = is_obstacle(node)
                if (not prev_wp_is_obstacle) and cur_wp_is_obstacle:
                    miniwp.append((prev_x, prev_y))
                elif prev_wp_is_obstacle and (not cur_wp_is_obstacle):
                    miniwp.append((current_x, current_y))
                prev_x = current_x
                prev_y = current_y
                current_x += 1
                prev_wp_is_obstacle = cur_wp_is_obstacle
        else:
            while current_x > gx:
                current_y = m*current_x + c
                node = Node((current_x/grid_size), (current_y/grid_size), 0, 0)
                cur_wp_is_obstacle = is_obstacle(node)
                if (not prev_wp_is_obstacle) and cur_wp_is_obstacle:
                    miniwp.append((prev_x, prev_y))
                elif prev_wp_is_obstacle and (not cur_wp_is_obstacle):
                    miniwp.append((current_x, current_y))
                prev_x = current_x
                prev_y = current_y
                current_x -= 1
                prev_wp_is_obstacle = cur_wp_is_obstacle

        if len(miniwp) % 2 != 0:
            final_length = len(miniwp) - 1
        else:
            final_length = len(miniwp)

        for k in range(0, final_length, 2):
            fwpx = []
            fwpy = []
            j = k+1
            sx = miniwp[k][0]
            sy = miniwp[k][1]
            gx = miniwp[j][0]
            gy = miniwp[j][1]
            minx = round(sx) - 100
            miny = round(sy) - 100
            maxx = round(gx) + 100
            maxy = round(gy) + 100

            rx, ry = a_star_planning(sx, sy, gx, gy, minx, miny, maxx, maxy, grid_size, drone_size)
            rx.reverse()
            ry.reverse()

            for w in range(1, len(rx)-1):
                if (rx[w] == rx[w-1]) and (rx[w] == rx[w+1]):
                    continue
                if (ry[w] == ry[w-1]) and (ry[w] == ry[w+1]):
                    continue
                if (abs(rx[w-1] - rx[w]) == abs(ry[w-1] - ry[w])) and (abs(rx[w] - rx[w+1]) == abs(ry[w] - ry[w+1])):
                    continue

                fwpx.append(rx[w-1])
                fwpy.append(ry[w-1])

            fwpx.append(rx[-1])
            fwpy.append(ry[-1])

            for x, y in zip(fwpx, fwpy):
                line = lines[f]
                line = line.split()
                geoCo = utm.to_latlon(x, y, utmCo[f][2], utmCo[f][3])
                line[1] = '0'
                line[3] = '82' # As code for spline waypoint is 82, for normal waypoint use 16 instead of 82
                line[8] = str(geoCo[0])
                line[9] = str(geoCo[1])
                outputfile.write(f"{str(Count)}\t{line[1]}\t{line[2]}\t{line[3]}\t{line[4]}\t{line[5]}\t{line[6]}\t{line[7]}\t{line[8]}\t{line[9]}\t{line[10]}\t{line[11]}\n")
                Count += 1

        line = lines[f+1]
        line = line.split()
        line[0] = Count
        outputfile.write(f"{str(Count)}\t{line[1]}\t{line[2]}\t{line[3]}\t{line[4]}\t{line[5]}\t{line[6]}\t{line[7]}\t{line[8]}\t{line[9]}\t{line[10]}\t{line[11]}\n")
                
    outputfile.close()

def draw_kml_circles(obx,oby,radius):
    kml = simplekml.Kml()
    for i in range(len(obx)):
        polycircle = polycircles.Polycircle(latitude=obx[i],
                            longitude=oby[i],
                            radius=radius[i],
                            number_of_vertices=100)

        pol = kml.newpolygon(name="obstacle"+str(i),outerboundaryis=polycircle.to_kml())
        pol.style.polystyle.color = simplekml.Color.changealphaint(radius[i], simplekml.Color.green)
    kml.save("./output/obs.kml")

def final():
    inter_wp_file_path = r"./inter/inter.waypoints"
    
    inter_wp_file = open(inter_wp_file_path, "r")
    
    final_op_wp_path = r"./output/new_mission.waypoints"
    outputfile = open(final_op_wp_path, "w")

    line = inter_wp_file.readline()
    outputfile.write(line)
    line = inter_wp_file.readline()

    lines = []
    while line:
        if (len(line) > 0):
            lines.append(line)
            line = line.split()
        line = inter_wp_file.readline()
    
    inter_waypoint(inter_wp_file_path)
    
    threshold_value = input("Enter the threshold value for the distacne between two waypoints (in Feet): ")
    
    def insert_center_wp():
        count = 0
        for i in range (len(inter_wp)):
            len_inter_wp = len(inter_wp)
            if i >= len_inter_wp-count-1:
                exit
            else:
                R = 20925525 # Alternate Value 20902230.97

                dlat = (inter_wp[i+1][0]-inter_wp[i][0])*math.pi/180
                dlng = (inter_wp[i+1][1]-inter_wp[i][1])*math.pi/180

                a = math.sin(dlat/2)**2 + math.cos(inter_wp[i][0])*math.cos(inter_wp[i+1][0])*math.sin(dlng/2)*math.sin(dlng/2)
                c = 2*math.atan2(math.sqrt(a), math.sqrt(1-a))

                distance = R * c

                # print(f"Result: {distance}")

                if float(distance) <= float(threshold_value):

                    count = count + 1
                    mid_lat, mid_lng = midpoint(inter_wp[i][0], inter_wp[i+1][0], inter_wp[i][1], inter_wp[i+1][1])

                    inter_wp.pop(i+1)
                    inter_wp.pop(i)
                    inter_wp.insert(i, [mid_lat, mid_lng])
                    
                # else:
                #     final_inter_wp.append(inter_wp[i])

    for i in range(100):
        insert_center_wp()

    Count = 0

    final_inter_wp = inter_wp

    for z in range (len(final_inter_wp)):
        line = lines[z]
        line = line.split()

        line[8] = str(final_inter_wp[z][0])
        line[9] = str(final_inter_wp[z][1])

        outputfile.write(f"{str(Count)}\t{line[1]}\t{line[2]}\t{line[3]}\t{line[4]}\t{line[5]}\t{line[6]}\t{line[7]}\t{line[8]}\t{line[9]}\t{line[10]}\t{line[11]}\n")
        Count =+ 1  

    print(f"Final waypoint file with Avoided Obstacles is saved in {final_op_wp_path} and KML file to visualise Obstacles.")

def main():
    draw_kml_circles(obx, oby, radiusint)
    inter()
    final()

if __name__ == "__main__":
    main()
