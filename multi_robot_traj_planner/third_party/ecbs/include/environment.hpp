//
// Modified by jungwon on 19. 1. 21.
// Add side constraint on 19. 5. 14.
// grid free on 19. 8. 29.
//

#ifndef SWARM_PLANNER_ENVIRONMENT_H
#define SWARM_PLANNER_ENVIRONMENT_H

#include <ecbs.hpp>
#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>
#include <math.h>

using namespace libMultiRobotPlanning;

namespace libMultiRobotPlanning {
    struct State {
        State(int time, int x, int y, int zz) : time(time), x(x), y(y), z(zz) {
            if (zz==4)
                z=0;
            if (zz==-1)
                z=3;
        }

        bool operator==(const State &s) const {
            return time == s.time && x == s.x && y == s.y && z == s.z;
        }

        State operator-(const State &s) const {
            return {time, x - s.x, y - s.y, z - s.z};
        }

        State operator+(const State &s) const {
            return {time, x + s.x, y + s.y, z + s.z};
        }

        bool equalExceptTime(const State &s) const { return x == s.x && y == s.y; }

        friend std::ostream &operator<<(std::ostream &os, const State &s) {
            return os << s.time << ": (" << s.x << "," << s.y << "," << s.z << ")";
            // return os << "(" << s.x << "," << s.y << "," << s.z <<")";
        }

        void set(int timer, int xr, int yr, int zr){
            x=xr; 
            y=yr;  
            time=timer;
            z=zr;
        }

        int time;
        int x;
        int y;
        int z;
    };

    struct Vector {
        Vector(double x, double y, double z) : x(x), y(y) {}
        Vector (const State& s): x((double)s.x), y((double)s.y) {}

        bool operator==(const Vector &v) const {
            return x == v.x && y == v.y;
        }

        Vector operator-(const Vector &v) const {
            return {x - v.x, y - v.y, 0};
        }

        double norm(){
            return sqrt(pow(x, 2) + pow(y, 2));
        }

        void normalize(){
            double vector_norm = norm();
            x /= vector_norm;
            y /= vector_norm;
        }

        double dot(const Vector& v){
            return x*v.x+y*v.y;
        }

        double min_dist_to_origin(Vector b){
            Vector a = *this;
            double min_dist = norm();

            if(!(a == b)) {
                double dist = b.norm();
                if (min_dist > dist) {
                    min_dist = dist;
                }

                Vector n = b - a;
                n.normalize();
                double a_dot_n = a.dot(n);
                n.x *= a_dot_n;
                n.y *= a_dot_n;
                Vector c = a - n;
                dist = c.norm();

                if ((c - a).dot(c - b) < 0 && min_dist > dist) {
                    min_dist = dist;
                }
            }
            return min_dist;
        }

        double x;
        double y;
    };
}

namespace std {
    template <>
    struct hash<State> {
        size_t operator()(const State& s) const {
            size_t seed = 0;
            boost::hash_combine(seed, s.time);
            boost::hash_combine(seed, s.x);
            boost::hash_combine(seed, s.y);
            boost::hash_combine(seed, s.z);
            return seed;
        }
    };
}  // namespace std

namespace libMultiRobotPlanning {
    enum class Action {
        Forward,
        Left,
        Right,
        Wait,
        ForwardLeft,
        ForwardRight
    };

    std::ostream &operator<<(std::ostream &os, const Action &a) {
        switch (a) {
            case Action::Forward:
                os << "Forward";
                break;
            case Action::Left:
                os << "Left";
                break;
            case Action::Right:
                os << "Right";
                break;
            case Action::Wait:
                os << "Wait";
                break;
            case Action::ForwardLeft:
                os << "ForwardLeft";
                break;
            case Action::ForwardRight:
                os << "ForwardRight";
                break;
        }
        return os;
    }

    struct Conflict {
        enum Type {
            Vertex,
            Edge,
        };

        int time;
        size_t agent1;
        size_t agent2;
        Type type;

        int x1;
        int y1;
        int z1;
        int x1_2;
        int y1_2;
        int z1_2;
        int x2;
        int y2;
        int z2;
        int x2_2;
        int y2_2;
        int z2_2;

        friend std::ostream &operator<<(std::ostream &os, const Conflict &c) {
            switch (c.type) {
                case Vertex:
                    return os << c.time << ": (" << c.agent1 << "," << c.agent2
                              << "): Vertex(" << c.x1 << "," << c.y1 << "," << c.z1 << ")";
                case Edge:
                    return os << c.time << ": (" << c.agent1 << "," << c.agent2
                              << "): Edge(" << c.x1 << "," << c.y1 << "," << c.z1 << ","
                              << c.x1_2 << "," << c.y1_2 << "," << c.z1_2 << ","
                              << c.x2 << "," << c.y2 << "," << c.z2 << ","
                              << c.x2_2 << "," << c.y2_2 << "," << c.z2_2 << ")";
            }
            return os;
        }
    };

    struct VertexConstraint {
        VertexConstraint(int time, int x, int y) : time(time), x(x), y(y) {}

        int time;
        int x;
        int y;

        bool operator<(const VertexConstraint &other) const {
            return std::tie(time, x, y) < std::tie(other.time, other.x, other.y);
        }

        bool operator==(const VertexConstraint &other) const {
            return std::tie(time, x, y) == std::tie(other.time, other.x, other.y);
        }

        friend std::ostream &operator<<(std::ostream &os, const VertexConstraint &c) {
            return os << "VC(" << c.time << "," << c.x << "," << c.y << ")";
        }
    };
}

namespace std {
    template <>
    struct hash<VertexConstraint> {
        size_t operator()(const VertexConstraint& s) const {
            size_t seed = 0;
            boost::hash_combine(seed, s.time);
            boost::hash_combine(seed, s.x);
            boost::hash_combine(seed, s.y);
            return seed;
        }
    };
}  // namespace std

namespace libMultiRobotPlanning {
    struct EdgeConstraint {
        EdgeConstraint(int time, int x1, int y1, int x2, int y2)
                : time(time), x1(x1), y1(y1), x2(x2), y2(y2) {}

        int time;
        int x1;
        int y1;
        int x2;
        int y2;

        bool operator<(const EdgeConstraint &other) const {
            return std::tie(time, x1, y1, x2, y2) <
                   std::tie(other.time, other.x1, other.y1, other.x2, other.y2);
        }

        bool operator==(const EdgeConstraint &other) const {
            return std::tie(time, x1, y1, x2, y2) ==
                   std::tie(other.time, other.x1, other.y1, other.x2, other.y2);
        }

        friend std::ostream &operator<<(std::ostream &os, const EdgeConstraint &c) {
            return os << "EC(" << c.time << "," << c.x1 << "," << c.y1 << "," << ","
                      << c.x2 << "," << c.y2 << ")";
        }
    };
}

namespace std {
    template <>
    struct hash<EdgeConstraint> {
        size_t operator()(const EdgeConstraint& s) const {
            size_t seed = 0;
            boost::hash_combine(seed, s.time);
            boost::hash_combine(seed, s.x1);
            boost::hash_combine(seed, s.y1);
            boost::hash_combine(seed, s.x2);
            boost::hash_combine(seed, s.y2);
            return seed;
        }
    };
}  // namespace std

namespace libMultiRobotPlanning {
    struct Constraints {
        std::unordered_set<VertexConstraint> vertexConstraints;
        std::unordered_set<EdgeConstraint> edgeConstraints;

        void add(const Constraints &other) {
            vertexConstraints.insert(other.vertexConstraints.begin(),
                                     other.vertexConstraints.end());
            edgeConstraints.insert(other.edgeConstraints.begin(),
                                   other.edgeConstraints.end());
        }

        bool overlap(const Constraints &other) {
            std::vector<VertexConstraint> vertexIntersection;
            std::vector<EdgeConstraint> edgeIntersection;
            std::set_intersection(vertexConstraints.begin(), vertexConstraints.end(),
                                  other.vertexConstraints.begin(),
                                  other.vertexConstraints.end(),
                                  std::back_inserter(vertexIntersection));
            std::set_intersection(edgeConstraints.begin(), edgeConstraints.end(),
                                  other.edgeConstraints.begin(),
                                  other.edgeConstraints.end(),
                                  std::back_inserter(edgeIntersection));
            return !vertexIntersection.empty() || !edgeIntersection.empty();
        }

        friend std::ostream &operator<<(std::ostream &os, const Constraints &c) {
            for (const auto &vc : c.vertexConstraints) {
                os << vc << std::endl;
            }
            for (const auto &ec : c.edgeConstraints) {
                os << ec << std::endl;
            }
            return os;
        }
    };

    struct Location {
        Location(int x, int y, int z) : x(x), y(y), z(z) {}

        int x;
        int y;
        int z;

        bool operator<(const Location &other) const {
            return std::tie(x, y) < std::tie(other.x, other.y);
        }

        bool operator==(const Location &other) const {
            return std::tie(x, y) == std::tie(other.x, other.y);
        }

        friend std::ostream &operator<<(std::ostream &os, const Location &c) {
            return os << "(" << c.x << "," << c.y << ")";
        }
    };
}
namespace std {
    template <>
    struct hash<Location> {
        size_t operator()(const Location& s) const {
            size_t seed = 0;
            boost::hash_combine(seed, s.x);
            boost::hash_combine(seed, s.y);
            return seed;
        }
    };
}  // namespace std

///
namespace libMultiRobotPlanning {
    class Environment {
    public:
        Environment(size_t dimx, size_t dimy, size_t dimz,
                    std::unordered_set<Location> obstacles,
                    std::vector<Location> goals,
                    std::vector<double> quad_size,
                    double grid_size)
                : m_dimx(dimx),
                  m_dimy(dimy),
                  m_dimz(dimz),
                  m_obstacles(std::move(obstacles)),
                  m_goals(std::move(goals)),
                  m_agentIdx(0),
                  m_constraints(nullptr),
                  m_lastGoalConstraint(-1),
                  m_highLevelExpanded(0),
                  m_lowLevelExpanded(0),
                  m_quad_size(std::move(quad_size)),
                  m_grid_size(grid_size) {}

        Environment(const Environment &) = delete;

        Environment &operator=(const Environment &) = delete;

        //find last goal constraint!
        void setLowLevelContext(size_t agentIdx, const Constraints *constraints) {
            assert(constraints);
            m_agentIdx = agentIdx;
            m_constraints = constraints;
            m_lastGoalConstraint = -1;
            for (const auto &vc : constraints->vertexConstraints) {
                if (vc.x == m_goals[m_agentIdx].x && vc.y == m_goals[m_agentIdx].y) {
                    m_lastGoalConstraint = std::max(m_lastGoalConstraint, vc.time);
                }
            }
        }

        int admissibleHeuristic(const State &s) {
            return std::abs(s.x - m_goals[m_agentIdx].x) +
                   std::abs(s.y - m_goals[m_agentIdx].y);
        }

        // low-level, get numConflict(equal state) from given solution
        int focalStateHeuristic(
                const State &s, int /*gScore*/,
                const std::vector<PlanResult<State, Action, int> > &solution) {
            int numConflicts = 0;
            for (size_t i = 0; i < solution.size(); ++i) {
                if (i != m_agentIdx && !solution[i].states.empty()) {
                    State state2 = getState(i, solution, s.time);
                    if (isVertexConflict(m_agentIdx, i, s, state2)) {
                        ++numConflicts;
                    }
                }
            }
            return numConflicts;
        }

        // low-level, get numConflict(s1a <-> s1b) from given solution
        int focalTransitionHeuristic(
                const State &s1a, const State &s1b, int /*gScoreS1a*/, int /*gScoreS1b*/,
                const std::vector<PlanResult<State, Action, int> > &solution) {
            int numConflicts = 0;
            for (size_t i = 0; i < solution.size(); ++i) {
                if (i != m_agentIdx && !solution[i].states.empty()) {
                    State s2a = getState(i, solution, s1a.time);
                    State s2b = getState(i, solution, s1b.time);
                    if (isEdgeConflict(m_agentIdx, i, s1a, s1b, s2a, s2b)) {
                        ++numConflicts;
                    }
                }
            }
            return numConflicts;
        }

        // Count all conflicts
        int focalHeuristic(
                const std::vector<PlanResult<State, Action, int> > &solution) {
            int numConflicts = 0;

            int max_t = 0;
            for (const auto &sol : solution) {
                max_t = std::max<int>(max_t, sol.states.size() - 1);
            }

            for (int t = 0; t < max_t; ++t) {
                // check drive-drive vertex collisions
                for (size_t i = 0; i < solution.size(); ++i) {
                    State state1 = getState(i, solution, t);
                    for (size_t j = i + 1; j < solution.size(); ++j) {
                        State state2 = getState(j, solution, t);
                        if (isVertexConflict(i, j, state1, state2)) {
                            ++numConflicts;
                        }
                    }
                }
                // drive-drive edge (swap)
                for (size_t i = 0; i < solution.size(); ++i) {
                    State state1a = getState(i, solution, t);
                    State state1b = getState(i, solution, t + 1);
                    for (size_t j = i + 1; j < solution.size(); ++j) {
                        State state2a = getState(j, solution, t);
                        State state2b = getState(j, solution, t + 1);
                        //TODO: fix it
                        if (isEdgeConflict(i, j, state1a, state1b, state2a, state2b)) {
                            ++numConflicts;
                        }
                    }
                }
            }
            return numConflicts;
        }

        bool isSolution(const State &s) {
            return s.x == m_goals[m_agentIdx].x && s.y == m_goals[m_agentIdx].y &&
                   s.time > m_lastGoalConstraint;
        }

        void getNeighbors(const State &s,
                          std::vector<Neighbor<State, Action, int> > &neighbors) {
            // std::cout << "#VC " << constraints.vertexConstraints.size() << std::endl;
            // for(const auto& vc : constraints.vertexConstraints) {
            //   std::cout << "  " << vc.time << "," << vc.x << "," << vc.y << "," << vc.z <<
            //   std::endl;
            // }
            neighbors.clear();
            {
                State n(s.time + 1, s.x, s.y, s.z);
                if (stateValid(n) && transitionValid(s, n)) {
                    neighbors.emplace_back(
                            Neighbor<State, Action, int>(n, Action::Wait, 1));
                }
            }
            {
                State n(s.time + 1, s.x, s.y, s.z + 1);
                if (stateValid(n) && transitionValid(s, n)) {
                    neighbors.emplace_back(
                            Neighbor<State, Action, int>(n, Action::Left, 2));
                }
            }
            {
                State n(s.time + 1, s.x, s.y, s.z - 1);
                if (stateValid(n) && transitionValid(s, n)) {
                    neighbors.emplace_back(
                            Neighbor<State, Action, int>(n, Action::Right, 2));
                }
            }
            {
                State n(0,0,0,0);
                switch (s.z){
                    case 0:
                        n.set(s.time + 1, s.x + 1, s.y, s.z);
                        break;
                    case 1:
                        n.set(s.time + 1, s.x, s.y + 1, s.z);
                        break;
                    case 2:
                        n.set(s.time + 1, s.x - 1, s.y, s.z);
                        break;
                    case 3:
                        n.set(s.time + 1, s.x, s.y - 1, s.z);
                        break;
                }
                if (stateValid(n) && transitionValid(s, n)) {
                    neighbors.emplace_back(
                            Neighbor<State, Action, int>(n, Action::Forward, 1));
                }
            }
            {
                State n(0,0,0,0);
                switch (s.z){
                    case 0:
                        n.set(s.time + 1, s.x + 1, s.y + 1, s.z+1);
                        break;
                    case 1:
                        n.set(s.time + 1, s.x - 1, s.y + 1, s.z+1);
                        break;
                    case 2:
                        n.set(s.time + 1, s.x - 1, s.y - 1, s.z+1);
                        break;
                    case 3:
                        n.set(s.time + 1, s.x + 1, s.y - 1, s.z-3);
                        break;
                }
                if (stateValid(n) && transitionValid(s, n)) {
                    neighbors.emplace_back(
                            Neighbor<State, Action, int>(n, Action::ForwardLeft, 2));
                }
            }
            {
                State n(0,0,0,0);
                switch (s.z){
                    case 0:
                        n.set(s.time + 1, s.x + 1, s.y - 1, s.z+3);
                        break;
                    case 1:
                        n.set(s.time + 1, s.x + 1, s.y + 1, s.z-1);
                        break;
                    case 2:
                        n.set(s.time + 1, s.x - 1, s.y + 1, s.z-1);
                        break;
                    case 3:
                        n.set(s.time + 1, s.x - 1, s.y - 1, s.z-1);
                        break;
                }
                if (stateValid(n) && transitionValid(s, n)) {
                    neighbors.emplace_back(
                            Neighbor<State, Action, int>(n, Action::ForwardRight, 2));
                }
            }
            /*
            {
                State n(s.time + 1, s.x, s.y, s.z + 1);
                if (stateValid(n) && transitionValid(s, n)) {
                    neighbors.emplace_back(
                            Neighbor<State, Action, int>(n, Action::Top, 1));
                }
            }
            {
                State n(s.time + 1, s.x, s.y, s.z - 1);
                if (stateValid(n) && transitionValid(s, n)) {
                    neighbors.emplace_back(
                            Neighbor<State, Action, int>(n, Action::Bottom, 1));
                }
            }
            */
        }

        bool getFirstConflict(
                const std::vector<PlanResult<State, Action, int> > &solution,
                Conflict &result) {
            int max_t = 0;
            for (const auto &sol : solution) {
                max_t = std::max<int>(max_t, sol.states.size() - 1);
            }

            for (int t = 0; t < max_t; ++t) {
                // check drive-drive vertex collisions
                for (size_t i = 0; i < solution.size(); ++i) {
                    State state1 = getState(i, solution, t);
                    for (size_t j = i + 1; j < solution.size(); ++j) {
                        State state2 = getState(j, solution, t);
                        if (isVertexConflict(i, j, state1, state2)) {
                            result.time = t;
                            result.agent1 = i;
                            result.agent2 = j;
                            result.type = Conflict::Vertex;
                            result.x1 = state1.x;
                            result.y1 = state1.y;
                            result.z1 = state1.z;
                            result.x2 = state2.x;
                            result.y2 = state2.y;
                            result.z2 = state2.z;
                            // std::cout << "VC " << t << "," << state1.x << "," << state1.y << "," << state1.z <<
                            // std::endl;
                            return true;
                        }
                    }
                }
                // drive-drive edge (swap)
                for (size_t i = 0; i < solution.size(); ++i) {
                    State state1a = getState(i, solution, t);
                    State state1b = getState(i, solution, t + 1);
                    for (size_t j = i + 1; j < solution.size(); ++j) {
                        State state2a = getState(j, solution, t);
                        State state2b = getState(j, solution, t + 1);

                        if (isEdgeConflict(i, j, state1a, state1b, state2a, state2b)) {
                            result.time = t;
                            result.agent1 = i;
                            result.agent2 = j;
                            result.type = Conflict::Edge;
                            result.x1 = state1a.x;
                            result.y1 = state1a.y;
                            result.z1 = state1a.z;
                            result.x1_2 = state1b.x;
                            result.y1_2 = state1b.y;
                            result.z1_2 = state1b.z;
                            result.x2 = state2a.x;
                            result.y2 = state2a.y;
                            result.z2 = state2a.z;
                            result.x2_2 = state2b.x;
                            result.y2_2 = state2b.y;
                            result.z2_2 = state2b.z;
                            return true;
                        }
                    }
                }
            }

            return false;
        }

        void createConstraintsFromConflict(
                const Conflict &conflict, std::map<size_t, Constraints> &constraints) {
            if (conflict.type == Conflict::Vertex) {
                Constraints c1, c2;
                c1.vertexConstraints.emplace(
                        VertexConstraint(conflict.time, conflict.x1, conflict.y1));
                c2.vertexConstraints.emplace(
                        VertexConstraint(conflict.time, conflict.x2, conflict.y2));
                constraints[conflict.agent1] = c1;
                constraints[conflict.agent2] = c2;
            } else if (conflict.type == Conflict::Edge) {
                Constraints c1, c2;
                c1.edgeConstraints.emplace(EdgeConstraint(
                        conflict.time, conflict.x1, conflict.y1, conflict.x1_2, conflict.y1_2));
                c2.edgeConstraints.emplace(EdgeConstraint(
                        conflict.time, conflict.x2, conflict.y2, conflict.x2_2, conflict.y2_2));
                constraints[conflict.agent1] = c1;
                constraints[conflict.agent2] = c2;
            }
        }

        void onExpandHighLevelNode(int /*cost*/) { m_highLevelExpanded++; }

        void onExpandLowLevelNode(const State & /*s*/, int /*fScore*/,
                                  int /*gScore*/) {
            m_lowLevelExpanded++;
        }

        int highLevelExpanded() { return m_highLevelExpanded; }

        int lowLevelExpanded() const { return m_lowLevelExpanded; }

    private:
        State getState(size_t agentIdx,
                       const std::vector<PlanResult<State, Action, int> > &solution,
                       size_t t) {
            assert(agentIdx < solution.size());
            if (t < solution[agentIdx].states.size()) {
                return solution[agentIdx].states[t].first;
            }
            assert(!solution[agentIdx].states.empty());
            return solution[agentIdx].states.back().first;
        }

        bool stateValid(const State &s) {
            assert(m_constraints);
            const auto &con = m_constraints->vertexConstraints;
            return s.x >= 0 && s.x < m_dimx && s.y >= 0 && s.y < m_dimy &&
                   m_obstacles.find(Location(s.x * 5, s.y * 5, s.z)) == m_obstacles.end() &&
                   con.find(VertexConstraint(s.time, s.x, s.y)) == con.end();
        }

        bool transitionValid(const State &s1, const State &s2) {
            assert(m_constraints);
            const auto &con = m_constraints->edgeConstraints;
            return m_obstacles.find(Location(s1.x * 2 + s2.x * 3, s1.y * 2 + s2.y * 3, s1.z)) == m_obstacles.end() &&
                   m_obstacles.find(Location(s1.x * 3 + s2.x * 2, s1.y * 3 + s2.y * 2, s1.z)) == m_obstacles.end() &&
                   m_obstacles.find(Location(s1.x * 1 + s2.x * 4, s1.y * 1 + s2.y * 4, s1.z)) == m_obstacles.end() &&
                   m_obstacles.find(Location(s1.x * 4 + s2.x * 1, s1.y * 4 + s2.y * 1, s1.z)) == m_obstacles.end() &&
                   con.find(EdgeConstraint(s1.time, s1.x, s1.y, s2.x, s2.y)) == con.end();
        }

        bool isParallel(const State &state1a, const State &state1b, const State &state2a, const State &state2b) {
            return (state1b.x - state1a.x) == (state2b.x - state2a.x) &&
                   (state1b.y - state1a.y) == (state2b.y - state2a.y);
        }

        bool isVertexConflict(int i, int j, const State &state1, const State &state2) {
            
            if (m_quad_size[i] + m_quad_size[j] < m_grid_size) {
                return state1.equalExceptTime(state2);
            }
            else{
                Vector v(state2 - state1);
                return v.norm() * m_grid_size < m_quad_size[i] + m_quad_size[j];
            }
            
            return false;
        }

        bool isEdgeConflict(int i, int j, const State &state1a, const State &state1b,
                                          const State &state2a, const State &state2b){
            
            if (m_quad_size[i] + m_quad_size[j] < m_grid_size * 0.5) {
                return (state1a.equalExceptTime(state2b) && state1b.equalExceptTime(state2a))
                ||((state1a+state1b).equalExceptTime(state2a+state2b));
            }
            else if(m_quad_size[i] + m_quad_size[j] < m_grid_size) {
                return (state1a.equalExceptTime(state2b) || state2a.equalExceptTime(state1b)) &&
                       !isParallel(state1a, state1b, state2a, state2b);
            }
            else{
                Vector a(state2a - state1a);
                Vector b(state2b - state1b);
                double min_dist = a.min_dist_to_origin(b);
                return min_dist * m_grid_size <= m_quad_size[i] + m_quad_size[j];
            }
            
            return false;
            
        }

    private:
        int m_dimx;
        int m_dimy;
        int m_dimz;
        std::unordered_set<Location> m_obstacles;
        std::vector<Location> m_goals;
        size_t m_agentIdx;
        const Constraints *m_constraints;
        int m_lastGoalConstraint;
        int m_highLevelExpanded;
        int m_lowLevelExpanded;
        std::vector<double> m_quad_size;
        double m_grid_size;
    };
}
#endif //SWARM_PLANNER_ENVIRONMENT_H
