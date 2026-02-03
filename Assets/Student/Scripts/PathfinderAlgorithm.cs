using System.Collections.Generic;
using UnityEngine;


public static class PathfindingAlgorithm
{
    // ======================
    // --- GRAPH CLASSES ---
    // ======================

    private class Edge
    {
        public Vector2Int To;
        public float Cost;

        public Edge(Vector2Int to, float cost)
        {
            To = to;
            Cost = cost;
        }
    }

    private class Graph
    {
        public Dictionary<Vector2Int, List<Edge>> Adjacency =
            new Dictionary<Vector2Int, List<Edge>>();
    }

    // ==========================================
    // --- GRAPH BUILDER (GENERATES ALL EDGES) ---
    // ==========================================

    private static Graph BuildGraph(IMapData map)
    {
        Graph graph = new Graph();

        for (int x = 0; x < map.Width; x++)
            for (int y = 0; y < map.Height; y++)
            {
                Vector2Int node = new Vector2Int(x, y);
                graph.Adjacency[node] = new List<Edge>();

                // Normal movement edges (grid neighbors)
                TryAddEdge(graph, map, node, new Vector2Int(x + 1, y)); // right
                TryAddEdge(graph, map, node, new Vector2Int(x - 1, y)); // left
                TryAddEdge(graph, map, node, new Vector2Int(x, y + 1)); // up
                TryAddEdge(graph, map, node, new Vector2Int(x, y - 1)); // down

                // Vent teleport edges
                if (map.HasVent(x, y))
                {
                    float ventCost = map.GetVentCost(x, y);
                    foreach (var otherVent in map.GetOtherVentPositions(node))
                    {
                        graph.Adjacency[node].Add(new Edge(otherVent, ventCost));
                    }
                }
            }

        return graph;
    }

    private static void TryAddEdge(Graph graph, IMapData map, Vector2Int from, Vector2Int to)
    {
        // Out of bounds
        if (to.x < 0 || to.y < 0 || to.x >= map.Width || to.y >= map.Height)
            return;

        // Compute cost using wall rules
        float cost = GetMovementCost(from, to, map);
        if (float.IsInfinity(cost))
            return; // wall blocks movement

        graph.Adjacency[from].Add(new Edge(to, cost));
    }

    private static float GetMovementCost(Vector2Int from, Vector2Int to, IMapData map)
    {
        int dx = to.x - from.x;
        int dy = to.y - from.y;

        // Horizontal movement → vertical wall between cells
        if (dx == 1)
            return map.GetVerticalWallCost(from.x + 1, from.y);
        if (dx == -1)
            return map.GetVerticalWallCost(from.x, from.y);

        // Vertical movement -> horizontal wall
        if (dy == 1)
            return map.GetHorizontalWallCost(from.x, from.y + 1);
        if (dy == -1)
            return map.GetHorizontalWallCost(from.x, from.y);

        return float.PositiveInfinity; // Should never happen
    }

    // =========================================
    // --- DIJKSTRA IMPLEMENTATION STARTS HERE ---
    // =========================================

    private class DNode : System.IComparable<DNode>
    {
        public Vector2Int Pos;
        public float Dist;

        public int CompareTo(DNode other)
        {
            int cmp = Dist.CompareTo(other.Dist);
            if (cmp != 0) return cmp;

            // Tie-breaker to avoid SortedSet collisions
            if (Pos.x != other.Pos.x) return Pos.x.CompareTo(other.Pos.x);
            return Pos.y.CompareTo(other.Pos.y);
        }
    }

    public static List<Vector2Int> FindShortestPath(Vector2Int start, Vector2Int goal, IMapData mapData)
    {
        Graph graph = BuildGraph(mapData);

        var dist = new Dictionary<Vector2Int, float>();
        var prev = new Dictionary<Vector2Int, Vector2Int>();
        var pq = new SortedSet<DNode>();

        dist[start] = 0f;
        pq.Add(new DNode { Pos = start, Dist = 0f });

        while (pq.Count > 0)
        {
            DNode current = pq.Min;
            pq.Remove(current);

            Vector2Int u = current.Pos;

            if (u == goal)
                return Reconstruct(prev, goal);

            foreach (var edge in graph.Adjacency[u])
            {
                float newDist = dist[u] + edge.Cost;

                if (!dist.ContainsKey(edge.To) || newDist < dist[edge.To])
                {
                    dist[edge.To] = newDist;
                    prev[edge.To] = u;

                    pq.Add(new DNode { Pos = edge.To, Dist = newDist });
                }
            }
        }

        Debug.LogWarning("No path found (Dijkstra).");
        return null;
    }

    private static List<Vector2Int> Reconstruct(
        Dictionary<Vector2Int, Vector2Int> prev,
        Vector2Int goal
    )
    {
        List<Vector2Int> path = new List<Vector2Int>();
        Vector2Int cur = goal;

        while (prev.ContainsKey(cur))
        {
            path.Add(cur);
            cur = prev[cur];
        }

        path.Add(cur);
        path.Reverse();
        return path;
    }

    // ===========================================
    // --- MOVEMENT BLOCKING (used by character) ---
    // ===========================================

    public static bool IsMovementBlocked(Vector2Int from, Vector2Int to, IMapData map)
    {
        // VENT TELEPORT CHECK — allow ANY non-adjacent movement
        if (map.HasVent(from.x, from.y))
        {
            foreach (var ventTarget in map.GetOtherVentPositions(from))
            {
                if (ventTarget == to)
                    return false; // teleport allowed
            }
        }

        // Otherwise use normal wall rules
        float cost = GetMovementCost(from, to, map);
        return float.IsInfinity(cost);
    }
}