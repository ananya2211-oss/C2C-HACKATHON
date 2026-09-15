"""
Azisly Hackathon -- Round 2 submission template.

Rename this file to your team id (e.g. team17.py) and submit it. One file, Python 3,
standard library only.

You edit ONE function: decide(). Everything below the DO NOT EDIT line handles talking
to the grader for you -- you never have to think about processes or JSON.

Read CONTRACT.md first. It is short and it is the whole ruleset.
"""

import json
import sys


def decide(sensors, memory):
    """Choose one action for this tick.

    sensors : dict -- this tick's readings. See CONTRACT.md for every field.
        sensors["dist_front"]  open cells ahead before a wall (0 = wall right there)
        sensors["dist_left"]   open cells to your left
        sensors["dist_right"]  open cells to your right
        sensors["rpm_left"]    left wheel speed from your PREVIOUS action
        sensors["rpm_right"]   right wheel speed from your PREVIOUS action
        sensors["accel_fwd"]   -2.0 means you just hit a wall
        sensors["accel_lat"]   +1.0 turned right, -1.0 turned left
        sensors["at_goal"]     True when you have arrived
        sensors["tick"]        tick counter

    memory : dict -- yours. It persists across ticks for the whole maze and starts
        empty. Put your map, your believed position, anything you like in here.

    Returns one of: "forward", "turn_left", "turn_right", "wait"

    ------------------------------------------------------------------------
    What is below is a RIGHT-HAND WALL FOLLOWER. It works: it will solve the
    early mazes. It is deliberately not good enough to win, because it has no
    memory -- it never learns the maze, so it walks the same long way round
    every time and it can loop forever in an open room.

    Your job is to do better. Some directions worth taking:

      1. Track where you are. The wheels tell you what you actually did:
         both wheels near +120 means you advanced one cell; wheels
         counter-rotating means you turned 90 degrees. Keep (x, y, heading)
         in memory and update it every tick.

      2. Build a map. Once you know where you are, record the walls you
         sense into memory. Now you know which parts of the maze you have
         not explored yet.

      3. Route with what you know. With a map, you can flood-fill or BFS to
         the nearest unexplored cell instead of wandering, and once you have
         found the goal you know the short way back.

      4. Handle the nasty mazes. On the hardest ones you only feel adjacent
         walls, distance readings are occasionally wrong by one, and the
         wheel encoders wobble by about 5. Compare RPM with a tolerance, not
         with ==, and consider ignoring a single odd sensor reading rather
         than trusting it immediately.

    Scoring, briefly: reaching the goal is worth far more than reaching it
    quickly, and every wall you hit costs you 5 points plus a wasted tick.
    Get it solving first. Optimise second.
    ------------------------------------------------------------------------
    """
    import collections

    HEADINGS = ("N", "E", "S", "W")
    DELTA = {"N": (0, -1), "E": (1, 0), "S": (0, 1), "W": (-1, 0)}

    # 1. State initialization
    if "edges" not in memory:
        memory["x"] = 0
        memory["y"] = 0
        memory["heading"] = "N"
        memory["visited"] = set()
        memory["edges"] = {}  # (x, y, h) -> "open" | "wall" | "hard_wall"
        memory["plan"] = []
        
    if "wall_seen" not in memory:
        memory["wall_seen"] = {}

    x = memory["x"]
    y = memory["y"]
    heading = memory["heading"]

    if sensors.get("at_goal"):
        return "wait"

    # 2. Update odometry from previous action
    if sensors["tick"] > 0:
        rpm_l = sensors["rpm_left"]
        rpm_r = sensors["rpm_right"]
        accel_fwd = sensors["accel_fwd"]
        accel_lat = sensors["accel_lat"]

        if accel_fwd <= -1.5:
            # Hard collision detected. Update map with permanent wall.
            memory["edges"][(x, y, heading)] = "hard_wall"
            dx, dy = DELTA[heading]
            reverse_h = HEADINGS[(HEADINGS.index(heading) + 2) % 4]
            memory["edges"][(x + dx, y + dy, reverse_h)] = "hard_wall"
        elif accel_lat > 0.5:
            heading = HEADINGS[(HEADINGS.index(heading) + 1) % 4]
        elif accel_lat < -0.5:
            heading = HEADINGS[(HEADINGS.index(heading) - 1) % 4]
        elif accel_fwd > 0.5 or (rpm_l > 20 and rpm_r > 20 and accel_fwd > 0.1):
            prev_x, prev_y = x, y
            dx, dy = DELTA[heading]
            x += dx
            y += dy
            # We explicitly know the path we just took is open!
            reverse_h = HEADINGS[(HEADINGS.index(heading) + 2) % 4]
            memory["edges"][(prev_x, prev_y, heading)] = "open"
            memory["edges"][(x, y, reverse_h)] = "open"

    memory["x"] = x
    memory["y"] = y
    memory["heading"] = heading
    memory["visited"].add((x, y))

    # 3. Update local map from sensors
    def relative_to_abs(rel_dir):
        idx = HEADINGS.index(heading)
        if rel_dir == "front": return HEADINGS[idx]
        if rel_dir == "right": return HEADINGS[(idx + 1) % 4]
        if rel_dir == "left":  return HEADINGS[(idx - 1) % 4]
        return HEADINGS[(idx + 2) % 4]

    for rel_dir in ("front", "left", "right"):
        dist = sensors[f"dist_{rel_dir}"]
        if dist < 0:
            dist = 0
        abs_h = relative_to_abs(rel_dir)
        dx, dy = DELTA[abs_h]
        nx, ny = x + dx, y + dy
        reverse_h = HEADINGS[(HEADINGS.index(abs_h) + 2) % 4]
        
        # We overwrite suspected walls ("wall") if new sensor data arrives,
        # but we NEVER overwrite a "hard_wall" generated by a collision.
        if dist == 0:
            if memory["edges"].get((x, y, abs_h)) != "hard_wall":
                memory["edges"][(x, y, abs_h)] = "wall"
                memory["wall_seen"][(x, y, abs_h)] = memory["wall_seen"].get((x, y, abs_h), 0) + 1
                memory["edges"][(nx, ny, reverse_h)] = "wall"
                memory["wall_seen"][(nx, ny, reverse_h)] = memory["wall_seen"].get((nx, ny, reverse_h), 0) + 1
        else:
            # The open cells
            for d in range(1, dist + 1):
                ox = x + dx * d
                oy = y + dy * d
                if memory["visited"].__contains__((ox, oy)):
                    continue
                if memory["edges"].get((ox - dx, oy - dy, abs_h)) != "hard_wall":
                    memory["edges"][(ox - dx, oy - dy, abs_h)] = "open"
                    memory["wall_seen"][(ox - dx, oy - dy, abs_h)] = 0
                    memory["edges"][(ox, oy, reverse_h)] = "open"
                    memory["wall_seen"][(ox, oy, reverse_h)] = 0
            
            # The distant wall
            if dist < 100:
                wall_x = x + dx * dist
                wall_y = y + dy * dist
                if memory["edges"].get((wall_x, wall_y, abs_h)) not in ("hard_wall", "open"):
                    memory["edges"][(wall_x, wall_y, abs_h)] = "wall"
                    if memory["wall_seen"].get((wall_x, wall_y, abs_h), 0) == 0:
                        memory["wall_seen"][(wall_x, wall_y, abs_h)] = 1
                    wall_reverse_x = wall_x + dx
                    wall_reverse_y = wall_y + dy
                    if memory["edges"].get((wall_reverse_x, wall_reverse_y, reverse_h)) not in ("hard_wall", "open"):
                        memory["edges"][(wall_reverse_x, wall_reverse_y, reverse_h)] = "wall"
                        if memory["wall_seen"].get((wall_reverse_x, wall_reverse_y, reverse_h), 0) == 0:
                            memory["wall_seen"][(wall_reverse_x, wall_reverse_y, reverse_h)] = 1

    # 4. Plan Validation against current sensors
    if memory["plan"]:
        valid = True
        sim_x, sim_y, sim_h = x, y, heading
        for action in memory["plan"]:
            if action == "turn_left":
                sim_h = HEADINGS[(HEADINGS.index(sim_h) - 1) % 4]
            elif action == "turn_right":
                sim_h = HEADINGS[(HEADINGS.index(sim_h) + 1) % 4]
            elif action == "forward":
                if memory["edges"].get((sim_x, sim_y, sim_h)) != "open":
                    valid = False
                    break
                dx, dy = DELTA[sim_h]
                sim_x += dx
                sim_y += dy
                
        if valid:
            action = memory["plan"][0]
            if action == "forward" and sensors["dist_front"] == 0:
                memory["plan"] = []
            else:
                return memory["plan"].pop(0)
        else:
            # The plan is compromised by new sensor information (or noise). Discard it.
            memory["plan"] = []



    # 6. Search known map for frontier
    while True:
        best_plan = None
        visited_state = set([(x, y, heading)])
        queue = [(x, y, heading, [])]

        while queue:
            cx, cy, ch, path = queue.pop(0)
            
            # Target condition: Is the current state facing an unexplored, open direction?
            if memory["edges"].get((cx, cy, ch)) == "open":
                dx, dy = DELTA[ch]
                nx, ny = cx + dx, cy + dy
                if (nx, ny) not in memory["visited"]:
                    best_plan = path
                    break
                    
            # Expand routing graph (ONLY moving through visited cells to reach the frontier)
            # Expand Forward
            if memory["edges"].get((cx, cy, ch)) == "open":
                dx, dy = DELTA[ch]
                nx, ny = cx + dx, cy + dy
                if (nx, ny) in memory["visited"]:
                    n_state = (nx, ny, ch)
                    if n_state not in visited_state:
                        visited_state.add(n_state)
                        queue.append((nx, ny, ch, path + ["forward"]))
                        
            # Expand Turns (turning in place is always safe)
            idx = HEADINGS.index(ch)
            left_h = HEADINGS[(idx - 1) % 4]
            l_state = (cx, cy, left_h)
            if l_state not in visited_state:
                visited_state.add(l_state)
                queue.append((cx, cy, left_h, path + ["turn_left"]))
                
            right_h = HEADINGS[(idx + 1) % 4]
            r_state = (cx, cy, right_h)
            if r_state not in visited_state:
                visited_state.add(r_state)
                queue.append((cx, cy, right_h, path + ["turn_right"]))
                
        if best_plan is not None:
            break
            
        # We are trapped! Forgive walls with increasing thresholds.
        forgiven = 0
        for threshold in range(1, 1000):
            for edge, count in list(memory.get("wall_seen", {}).items()):
                if count <= threshold and memory["edges"].get(edge) == "wall":
                    memory["edges"][edge] = "open"
                    memory["wall_seen"][edge] = threshold + 1
                    forgiven += 1
            if forgiven > 0:
                break
                
        if forgiven == 0:
            break
            
    if best_plan is not None:
        if not best_plan:
            action = "forward"
        else:
            memory["plan"] = best_plan
            action = memory["plan"].pop(0)
            
        # Final safety sanity check
        if action == "forward" and sensors["dist_front"] == 0:
            memory["plan"] = []
            return "turn_left"
        return action

    # 7. Fallback if stuck
    # If the map is fully explored or false walls isolated us, rotating in place
    # allows sensors to resample the environment without risking a collision.
    return "turn_left"


# =============================================================================
# DO NOT EDIT BELOW THIS LINE
# This is the plumbing that talks to the grader. Changing it will break your
# submission and score you zero.
# =============================================================================

def _main():
    print(json.dumps({"ready": True}), flush=True)
    memory = {}
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        sensors = json.loads(line)
        action = decide(sensors, memory)
        print(json.dumps({"action": action}), flush=True)
        if sensors.get("at_goal"):
            break


if __name__ == "__main__":
    _main()
