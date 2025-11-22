using System.Collections.Generic;
using UnityEngine;

public static class PathfindingAlgorithm
{
    // Directions for 4-neighbour movement
    private static readonly Vector2Int[] Directions = new[]
    {
        new Vector2Int(1, 0),
        new Vector2Int(-1, 0),
        new Vector2Int(0, 1),
        new Vector2Int(0, -1)
    };

    // Record type for priority queue
    private class NodeRecord
    {
        public Vector2Int Position;
        public float Cost;
        public int Id; // tie-breaker so SortedSet allows duplicates

        public NodeRecord(Vector2Int pos, float cost, int id)
        {
            Position = pos;
            Cost = cost;
            Id = id;
        }
    }

    // Compare nodes by cost, then by id
    private class NodeRecordComparer : IComparer<NodeRecord>
    {
        public int Compare(NodeRecord a, NodeRecord b)
        {
            int c = a.Cost.CompareTo(b.Cost);
            if (c != 0) return c;
            return a.Id.CompareTo(b.Id);
        }
    }

    public static List<Vector2Int> FindShortestPath(Vector2Int start, Vector2Int goal, IMapData mapData)
    {
        // Safety: check inside bounds
        if (!IsInside(start, mapData) || !IsInside(goal, mapData))
        {
            Debug.LogError("Start or goal is outside map bounds.");
            return null;
        }

        var dist = new Dictionary<Vector2Int, float>();
        var prev = new Dictionary<Vector2Int, Vector2Int>();

        var open = new SortedSet<NodeRecord>(new NodeRecordComparer());
        int idCounter = 0;

        dist[start] = 0f;
        open.Add(new NodeRecord(start, 0f, idCounter++));

        while (open.Count > 0)
        {
            // Get node with minimum distance
            NodeRecord currentRecord = open.Min;
            open.Remove(currentRecord);
            Vector2Int current = currentRecord.Position;

            // If this is an outdated record, skip it
            if (!dist.TryGetValue(current, out float currentDist) || currentRecord.Cost > currentDist)
                continue;

            // If we reached the goal, we can stop
            if (current == goal)
                break;

            // ----- 1) Normal 4-direction neighbours -----
            foreach (var dir in Directions)
            {
                Vector2Int neighbor = current + dir;
                if (!IsInside(neighbor, mapData))
                    continue;

                float stepCost = GetStepCost(current, neighbor, mapData);
                if (float.IsPositiveInfinity(stepCost))
                    continue; // impassable (e.g. wall with infinite cost)

                float newDist = currentDist + stepCost;

                if (!dist.TryGetValue(neighbor, out float oldDist) || newDist < oldDist)
                {
                    dist[neighbor] = newDist;
                    prev[neighbor] = current;
                    open.Add(new NodeRecord(neighbor, newDist, idCounter++));
                }
            }

            // ----- 2) Vent teleportation edges -----
            if (mapData.HasVent(current.x, current.y))
            {
                float ventCost = mapData.GetVentCost(current.x, current.y);
                if (!float.IsPositiveInfinity(ventCost))
                {
                    List<Vector2Int> otherVents = mapData.GetOtherVentPositions(current);
                    foreach (var targetVent in otherVents)
                    {
                        float newDist = currentDist + ventCost;

                        if (!dist.TryGetValue(targetVent, out float oldDist) || newDist < oldDist)
                        {
                            dist[targetVent] = newDist;
                            prev[targetVent] = current;
                            open.Add(new NodeRecord(targetVent, newDist, idCounter++));
                        }
                    }
                }
            }
        }

        // No reachable goal?
        if (!dist.ContainsKey(goal) || float.IsPositiveInfinity(dist[goal]))
        {
            Debug.LogWarning("No path found between start and goal.");
            return null;
        }

        // ----- Reconstruct path from goal back to start -----
        List<Vector2Int> path = new List<Vector2Int>();
        Vector2Int node = goal;
        path.Add(node);

        while (node != start)
        {
            node = prev[node];
            path.Add(node);
        }

        path.Reverse();
        return path;
    }

    /// <summary>
    /// Returns true if movement from 'from' to 'to' is blocked
    /// (outside map or wall with infinite cost).
    /// Used by GridCharacterMovement and can also help during BFS-steget.
    /// </summary>
    public static bool IsMovementBlocked(Vector2Int from, Vector2Int to, IMapData mapData)
    {
        // Outside the map? Always blocked.
        if (!IsInside(to, mapData))
            return true;

        int dx = Mathf.Abs(to.x - from.x);
        int dy = Mathf.Abs(to.y - from.y);

        // --- Vent teleport: allow non-adjacent move ONLY if both are vents ---
        if (dx + dy > 1)
        {
            // If both cells are vents, this is a legal teleport move.
            if (mapData.HasVent(from.x, from.y) && mapData.HasVent(to.x, to.y))
                return false;

            // Any other non-adjacent move is illegal.
            return true;
        }

        // --- Normal neighbour step: use wall/cost logic ---
        float stepCost = GetStepCost(from, to, mapData);
        return float.IsPositiveInfinity(stepCost);
    }


    // ----- Helpers -----

    private static bool IsInside(Vector2Int p, IMapData mapData)
    {
        return p.x >= 0 && p.x < mapData.Width &&
               p.y >= 0 && p.y < mapData.Height;
    }

    /// <summary>
    /// Cost for a single step between two neighbouring cells.
    /// Handles:
    ///  - base movement cost 1
    ///  - extra cost for walls with finite cost
    ///  - infinite cost for walls that are not climbable
    /// NOTE: this is for normal steps only; vent teleportation is handled separately.
    /// </summary>
    private static float GetStepCost(Vector2Int from, Vector2Int to, IMapData mapData)
    {
        int dx = to.x - from.x;
        int dy = to.y - from.y;

        // Only allow 4-neighbour moves here
        if (Mathf.Abs(dx) + Mathf.Abs(dy) != 1)
            return float.PositiveInfinity;

        float baseCost = 1.0f;

        // Horizontal movement -> vertical walls
        if (dx != 0)
        {
            int wallX = dx > 0 ? to.x : from.x;
            int wallY = from.y;

            float wallCost = mapData.GetVerticalWallCost(wallX, wallY);

            if (float.IsPositiveInfinity(wallCost))
                return float.PositiveInfinity; // wall cannot be climbed

            if (wallCost > 1.0f)
                baseCost += wallCost - 1.0f;   // match PathfindingManager logic
        }

        // Vertical movement -> horizontal walls
        if (dy != 0)
        {
            int wallX = from.x;
            int wallY = dy > 0 ? to.y : from.y;

            float wallCost = mapData.GetHorizontalWallCost(wallX, wallY);

            if (float.IsPositiveInfinity(wallCost))
                return float.PositiveInfinity;

            if (wallCost > 1.0f)
                baseCost += wallCost - 1.0f;
        }

        return baseCost;
    }
}
