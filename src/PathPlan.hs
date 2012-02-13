-- Copyright 2012 Ondrej Sykora
--
-- This file is part of PathPlan.
--
-- PathPlan is free software: you can redistribute it and/or modify
-- it under the terms of the GNU General Public License as published by
-- the Free Software Foundation, either version 3 of the License, or
-- (at your option) any later version.
--
-- PathPlan is distributed in the hope that it will be useful,
-- but WITHOUT ANY WARRANTY; without even the implied warranty of
-- MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
-- GNU General Public License for more details.
--
-- You should have received a copy of the GNU General Public License
-- along with PathPlan. If not, see <http://www.gnu.org/licenses/>.


-- A path planning algorithm for multiple agents in a 2D world. The idea
-- of the planner is based on the GraphPlan algorithm (for more details
-- on the algorithm, see http://www.cs.cmu.edu/~avrim/graphplan.html). The
-- first builds a planning graph and then tries to extract the path for
-- all agents.
--
-- In the current implementation, the path extraction is implemented the
-- simplest possible way by backtracking without caching conflicts
-- discovered during plan extraction. Also, the detection of situations
-- where no solution exists is implemented by limiting the depth of the
-- graph with the number of cells on the map. This is not an effective
-- bound (and I'm not quite sure it really is a bound, but it should work
-- OK for most cases).

module PathPlan where

import Control.Monad
import Data.Array
import Data.Ix
import Data.List
import qualified Data.ByteString.Lazy as BS
import qualified Data.Set as Set
import Debug.Trace
import System.IO

-- | A coodrinates of a single map cell.
type MapPoint = (Int, Int)
-- | A complete map (a rectangluar area of square cells, which are
--  either accessible or not accessible). The map is implemented as
--  a 2D array indexed by MapPoint types; the array contains True if
--  the cell is accessible; it contains False if the cell is not
--  accessible.
type MapCells = Array MapPoint Bool

-- | The type for ID's of the agents in the path planning problem.
newtype AgentId = AgentId Int deriving (Eq, Ord, Read, Show)

-- | The position of the agent on the map. The position is implemented
-- as a record that contains the ID of the agent, and its position.
data Position = Pos AgentId MapPoint deriving (Eq, Ord, Read, Show)
-- | The set of positions of robots on the map.
type Positions = Set.Set Position
-- | The set of conflicting positions.
type PositionConflicts = Set.Set (Position, Position)
-- | The layer of the planning graph that contains the acheivable positions
-- of the agents.
data PositionLayer = PL MapCells Positions PositionConflicts deriving (Eq, Read, Show)

-- | Contains information about a single node.
data Move = Mv AgentId MapPoint MapPoint deriving (Eq, Ord, Read, Show)
-- | The set of moves.
type Moves = Set.Set Move
-- | The set of conflicting moves.
type MoveConflicts = Set.Set (Move, Move)
-- | The layer of the planning graph that contains the moves.
data MoveLayer = ML MapCells Moves MoveConflicts deriving (Read, Show)

-- | The list of supports for the given position.
data PositionSupport = PS Position [Move] deriving (Eq, Show)
instance Ord PositionSupport where
    -- When ordering position supports, only the position and the ID of
    -- the agent is considered.
    compare (PS pos1 _) (PS pos2 _) = compare pos1 pos2

-- | Contains information about the pathfinding problem for a single
--  agent, i.e. the initial position and the destination of the agent.
data AgentPath = Agent MapPoint MapPoint deriving (Eq, Ord, Read, Show)
-- | Contains the specification of the pathfinding problem. Contains the
--  map and the initial positions and destinations of all agents.
data PathProblem = PP MapCells [AgentPath] deriving Show
-- | The pathfinding graph.
data PathFinding = PF PositionLayer [(MoveLayer, PositionLayer)] deriving Show

infixl 0 |>
x |> f = f x

-- | Sorts the items in a list according to a key provided by a function.
sortByKey :: Ord key => (a -> key) -> [a] -> [a]
sortByKey get_key items = sortBy compareKeys items
    where
        compareKeys a1 a2 = compare (get_key a1) (get_key a2)

-- | Calculates the number of elements on the map.
num_positions_on_map :: MapCells -> Integer
num_positions_on_map map = width * height
    where
        ((left, top), (right, bottom)) = bounds map
        width = fromIntegral (right - left + 1)
        height = fromIntegral (bottom - top + 1)

-- | Checks if the given point on the map is accessible (i.e. that there
--  is not a wall.
is_free :: MapCells -> (Int, Int) -> Bool
is_free map (x, y)
    | not $ inRange bnd (x, y)  = False
    | otherwise                 = map!(x, y)
    where
        bnd = bounds map

-- | Splits the given list by the given element. It can be used for example
--  to split string into lines, etc.
split :: Eq a => a -> [a] -> [[a]]
split _ [] = []
split d xs =
    let (hd, tl) = break (== d) xs in
    case tl of
        [] -> []
        (r:rs) -> hd:(split d rs)

-- | Creates the list of pairs of elements from the given list. Each pair
-- is generated once, the "lower" item (using the functions from Ord) is
-- always first in the pair. Only works for finite lists.
pairs :: Ord a => [a] -> [(a, a)]
pairs xs = concatMap (make_pairs) (make_list xs)
    where
        make_list [] = []
        make_list (x:xs) = (x, xs):(make_list xs)
        make_pairs (x, xs) = map (make_pair x) xs
        make_pair a b
            | a < b     = (a, b)
            | otherwise = (b, a)

-- | Creates the cross-product of two lists. Only works for finite lists.
cross :: [a] -> [b] -> [(a, b)]
cross xs ys = concatMap (make_pairs ys) xs
    where
        make_pairs :: [b] -> a -> [(a, b)]
        make_pairs ys x = map (\y -> (x, y)) ys

-- | Splits the list of agent positions into a list of agent IDs and a
-- list of coordinates.
split_positions :: [Position] -> ([AgentId], [MapPoint])
split_positions ps = unzip $ map split_single ps
    where
        split_single (Pos id pos) = (id, pos)

-- | Checks if there is an element that is more than once in the list.
has_duplicate :: (Ord a) => [a] -> Bool
has_duplicate [] = False
has_duplicate xs =
    any (\(a, b) -> a == b) zipped
    where
        zipped = zip sorted (tail sorted)
        sorted = sort xs

-- | Extracts the agent ID from a move specification.
get_move_agent_id :: Move -> AgentId
get_move_agent_id (Mv agent_id _ _) = agent_id

-- | Extracts the agent ID from a position specification.
get_position_agent_id :: Position -> AgentId
get_position_agent_id (Pos agent_id _) = agent_id

-- | Checks if the move starts at the given position.
starts_on_position :: MapPoint -> Move -> Bool
starts_on_position pos (Mv _ start _) = pos == start

-- | Checks if the move ends at the given position.
ends_on_position :: MapPoint -> Move -> Bool
ends_on_position pos (Mv _ _ end) = pos == end

-- | Extracts the list of target positions from the list of moves.
extract_positions_from_moves :: [Move] -> [Position]
extract_positions_from_moves moves = map (extract_position) moves
    where
        extract_position (Mv agent_id from to) = Pos agent_id from

-- | Builds a layer of the planning graphs that contains possible moves
-- based on the 'position layer'.
build_move_layer :: PositionLayer -> MoveLayer
build_move_layer (PL cells prev_positions prev_conflicts) =
    ML cells moves conflicts
    where
        -- Builds the set of possible moves based on the list of positions
        moves = Set.fromList moves_list
        moves_list = Set.fold (add_possible_moves) [] prev_positions
        -- Adds all possible moves created from the given position to the
        -- list of positions.
        add_possible_moves :: Position -> [Move] -> [Move]
        add_possible_moves prev@(Pos id (x, y)) moves =
            add_possible_move prev (Pos id (x, y)) $
            add_possible_move prev (Pos id (x + 1, y)) $
            add_possible_move prev (Pos id (x - 1, y)) $
            add_possible_move prev (Pos id (x, y - 1)) $
            add_possible_move prev (Pos id (x, y + 1)) moves
        add_possible_move :: Position -> Position -> [Move] -> [Move]
        add_possible_move (Pos id1 prev) (Pos id2 next) moves
            | id1 /= id2            = error "The positions must be for the same agent"
            | is_free cells next    = (Mv id1 prev next):moves
            | otherwise             = moves
        -- The set of conflicting moves
        conflicts = Set.fromList $ filter (is_conflict) $ pairs moves_list
        -- Checks if there is a conflict between the two given moves.
        is_conflict :: (Move, Move) -> Bool
        is_conflict (m1@(Mv id_a pos_a1 pos_a2), m2@(Mv id_b pos_b1 pos_b2))
            -- A move does not have a conflict with itself
            | m1 == m2                  = False
            -- An agent can have only one move
            | id_a == id_b              = True
            -- Only one agent can move from the given position
            | pos_a1 == pos_b1          = True
            | pos_a2 == pos_b2          = True
            | (pos_a1 == pos_b2)
                || (pos_a2 == pos_b1)   = True
            | otherwise                 = False

-- | Builds a layer of the planning graph that contains possible positions
-- based on the previous 'move layer'.
build_position_layer :: MoveLayer -> PositionLayer
build_position_layer (ML cells moves move_conflicts) =
    PL cells positions conflicts
    where
        -- Extracts the set of positions from the list of possible moves
        positions = extract_positions supports
        conflicts = extract_conflicts supports
        -- The "supports" for the positions in the layer. This are the moves,
        -- that created the positions. The moves are used for finding the
        -- conflicts between the positions.
        supports :: [PositionSupport]
        supports = Set.toList moves |>
                    sortBy (order_by_destination) |>
                    groupBy (same_destination) |>
                    make_supports

        -- Extracts the positions from the list of position supports.
        extract_positions :: [PositionSupport] -> Positions
        extract_positions ps = Set.fromList $ map (\(PS pos _) -> pos) ps

        -- Creates the list of position supports from the list of moves
        -- grouped by the destination position and agent id.
        make_supports :: [[Move]] -> [PositionSupport]
        make_supports xs = map (make_support) xs
        
        -- Creates a single "position support" from the given list of moves
        -- with the same destination position and agent id.
        make_support :: [Move] -> PositionSupport
        make_support ms = PS (Pos id pos) ms
            where (Mv id _ pos) = head ms

        -- Comparison function for moves, that compares two moves by their
        -- destination and agent id.
        order_by_destination :: Move -> Move -> Ordering
        order_by_destination (Mv id1 _ pos1) (Mv id2 _ pos2)
            | id1 == id2            = compare pos1 pos2
            | otherwise             = compare id1 id2
        -- Checks if the two moves have the same destination position and
        -- agent ID.
        same_destination :: Move -> Move -> Bool
        same_destination (Mv id1 _ pos1) (Mv id2 _ pos2) =
            (id1 == id2) && (pos1 == pos2)
        -- Extracts the conflicting positions based on the supports of the
        -- positions.
        extract_conflicts :: [PositionSupport] -> PositionConflicts
        extract_conflicts ss = pairs ss |> 
                                filter (is_conflict) |> 
                                map make_conflict_pair |>
                                Set.fromList
        make_conflict_pair :: (PositionSupport, PositionSupport) -> (Position, Position)
        make_conflict_pair (PS pos1 _, PS pos2 _)
            | pos1 < pos2   = (pos1, pos2)
            | otherwise     = (pos2, pos1)

        -- Checks if two position supports are in conflict, i.e. they are
        -- sending two different agents to the same position, sending one
        -- agent to two different positions, or all moves to the positions
        -- are in conflict.
        is_conflict :: (PositionSupport, PositionSupport) -> Bool
        is_conflict (PS p1@(Pos id1 pos1) mv1, PS p2@(Pos id2 pos2) mv2)
            | p1 == p2              = False
            | id1 == id2            = True
            | pos1 == pos2          = True
            | otherwise             = not $ any (not . is_move_conflict) $ cross mv1 mv2
        is_move_conflict :: (Move, Move) -> Bool
        is_move_conflict (m1, m2)
            | m1 == m2  = False
            | m1 < m2   = Set.member (m1, m2) move_conflicts
            | m1 > m2   = Set.member (m2, m1) move_conflicts

-- | Builds an initial layer of the planning graph. This is the position
-- layer with the initial positions of the agents.
build_initial_layer :: MapCells -> [Position] -> PositionLayer
build_initial_layer cells positions
    | is_conflict   = error "The initial positions contain conflicts"
    | otherwise     = PL cells position_set conflicts
    where
        is_conflict = (has_duplicate pos_points) || (has_duplicate pos_ids)
        position_set = Set.fromList positions
        conflicts = Set.empty
        (pos_points, pos_ids) = split_positions positions

find_path :: PositionLayer -> [Position] -> Maybe [[Move]]
find_path initial goals = build_layers 1 [] initial
    where
        build_layers :: Integer -> [(MoveLayer, PositionLayer)] -> PositionLayer -> Maybe [[Move]]
        build_layers depth reverse_layers_acc prev_pos_layer =
            trace ("Building a layer from" ++ (show prev_pos_layer) ++ "\n\n") $
            if is_final pos_layer
              then extracted_path
              else trace ("Path not found at depth " ++ (show depth) ++ "\n\n")$
                   build_layers (depth + 1) new_reverse_layers_acc pos_layer
            where
                new_reverse_layers_acc = (move_layer, pos_layer):reverse_layers_acc
                move_layer = build_move_layer prev_pos_layer
                pos_layer = build_position_layer move_layer
                is_final (PL _ positions conflicts) = (contains_goals
                                                        && no_goal_conflicts
                                                        && extracted_path /= Nothing)
                                                      || max_path_length < depth
                    where
                        contains_goals = all (\pos -> Set.member pos positions) goals
                        no_goal_conflicts = all (no_conflict) $ pairs goals
                            where
                                no_conflict (pos1, pos2)
                                    | pos1 == pos2      = False
                                    | otherwise         = not$ Set.member (pos1, pos2) conflicts
                extracted_path = extract_paths new_reverse_layers_acc goals
        max_path_length = num_agents * (num_positions_on_map the_map)
            where
                num_agents = fromIntegral$ length goals
                PL the_map _ _ = initial

build_positions :: MapCells -> [AgentPath] -> ([Position], [Position])
build_positions map_cells agents =
    unzip $ map (make_position) (zip (map (AgentId) [1..]) agents)
    where make_position :: (AgentId, AgentPath) -> (Position, Position)
          make_position (n, Agent start final)
                | (not$ map_cells!start) = (error$ "The initial position of agent " ++ (show n) ++ " is blocked")
                | (not$ map_cells!final) = (error$ "The final position of agent " ++ (show n) ++ " is blocked")
                | otherwise         = (Pos n start, Pos n final)

-- | Partitions the elements of the list 'items' by the value returned by the
-- function key_function. The result of the operation is a partitioning of items
-- into groups, where the items in each groups have the same value of key_function.
--
-- Example:
-- > partitionBy (fst) [(1, 2), (2, 3), (1, 4), (2, 5)]
-- > [[(1, 2), (1, 4)], [(2, 3), (2, 5)]]
partitionBy :: Ord b => (a -> b) -> [a] -> [[a]]
partitionBy key_function items =
    groupBy key_equals $ sortBy compare_key $ items
    where
        compare_key x y = compare (key_function x) (key_function y)
        key_equals x y = (key_function x) == (key_function y)

-- | Generates the list of all combinations of items from the given list of
-- sublists. The result is a list of sublists, where in each sublist each
-- element is taken from a different sublist in the original list. Assumes
-- that both the outer list and the sublists contain a finite number of
-- elements.
--
-- Example:
-- > combinations [[1, 2], [3, 4]]
-- > [[1, 3], [1, 4], [2, 3], [2, 4]]
combinations :: [[a]] -> [[a]]
combinations lists =
    combinations' [] lists
    where
        combinations' acc [] = do
            return $ reverse acc
        combinations' acc (l:ls) = do
            item <- l
            combinations' (item:acc) ls

-- | Extracts paths for the agents from the planning graph. If a non-conflicting
-- path for all agents is found, this path is returned as a list of moves. The
-- moves in the list are partially ordered. There is the same amount of moves
-- for each agent, regardless of the length of their path on the map; some of the
-- moves may be no-op moves (and they might be important for synchronizing the
-- paths).
-- If no such path was found, this method returns None.
extract_paths :: [(MoveLayer, PositionLayer)] -> [Position] -> Maybe [[Move]]
extract_paths reverse_layers goals =
    case possible_paths of
        []             -> Nothing
        paths@(path:_) -> trace ("Available paths: " ++ (show paths) ++ "\n\n") $ Just path
    where
        possible_paths = extract_paths' goals reverse_layers []
        -- | Extracts paths for the current positions of the agents.
        extract_paths' :: [Position] -> [(MoveLayer, PositionLayer)] -> [[Move]] -> [[[Move]]]
        extract_paths' [] _ _ = []
        extract_paths' _ [] moves = do
            return $ moves
        extract_paths' current_positions layers moves = do
            move_combination <- possible_move_combinations
            if has_move_conflicts move_combination
              then fail "Conflicts in the moves"
              else do
                let new_positions = extract_positions_from_moves move_combination
                extract_paths' new_positions next_layers (move_combination:moves)
            where
                has_move_conflicts move_list = any (is_move_conflict) (pairs move_list)
                is_move_conflict move_pair = Set.member move_pair move_conflicts

                possible_move_combinations = combinations available_moves_by_agent

                available_moves_by_agent = map (extract_available_moves) $ moves_with_positions

                extract_available_moves (Pos _ pos, moves) = filter (ends_on_position pos) moves

                moves_with_positions = zip sorted_positions all_moves_by_agent
                sorted_positions = sortByKey (get_position_agent_id) current_positions

                all_moves_by_agent = partitionBy (get_move_agent_id) (Set.toList all_moves)
                ML _ all_moves move_conflicts = current_move_layer
                (current_move_layer, current_pos_layer):next_layers = layers

-- | Loads the initial positions and the destinations of the agents from
--  the standard input, as well as the map. Returns an instance of the
--  pathfinding problem created from the input.
load_map :: IO PathProblem
load_map = do
    agent_count <- readLn :: IO Int
    agents <- read_agents agent_count
    raw_data <- getContents
    let lines = filter (not . null) $ split '\n' $ raw_data
    let bools = map (map (/= '#')) lines
    let numbered = zip [1..] bools
    let assoc = concatMap (\(y, ls) -> zip [(x, y) | x <- [1..]] ls) numbered
    let bounds = foldl (\(mx, my) ((x, y), _) -> (max x mx, max y my)) (0, 0) assoc
    let map = array ((1, 1), bounds) assoc
    return $ PP map agents
    where read_agents 0 = return []
          read_agents n = do
                a <- readLn :: IO AgentPath
                as <- read_agents (n - 1)
                return (a:as)

main = do
    PP map agents <- load_map
    let (positions, goals) = build_positions map agents
    let initial_layer = build_initial_layer map positions
    let path = find_path initial_layer goals
    hPutStrLn stderr (show initial_layer)
    case path of
        Nothing   -> hPutStrLn stderr "No path was found"
        Just p    -> print p
