using System.Collections;
using System.Collections.Generic;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

public class Kim : CharacterController
{
    [Tooltip("Radius to scan for zombies and finish")]
    [SerializeField] float ContextRadius = 15f;
    [Tooltip("Seconds between path recalculations")]
    [SerializeField] float PathUpdateInterval = 0.2f;
    [Tooltip("Draw penalty/avoidance gizmos in Scene view")]
    [SerializeField] bool ShowPenaltyVisualization = true;
    [Tooltip("Tile radius around Kim to visualize penalties")]
    [SerializeField] float VisualizationRange = 20f;
    [Tooltip("Cost added at the hard avoid edge")]
    [SerializeField] float SoftPenalty = 20f;
    [Tooltip("Inside this distance tiles are blocked")]
    [SerializeField] float HardAvoidDistance = 1.0f;
    [Tooltip("Penalty fades in between this and hard avoid distance")]
    [SerializeField] float SoftAvoidDistance = 2.5f;

    private float pathUpdateTimer = 0f;
    private List<Grid.Tile> currentPath = new();
    private List<Zombie> zombiesInContext = new();
    private List<Burger> burgersInScene = new();
    private bool isFleeing = false;

    public override void StartCharacter()
    {
        base.StartCharacter();
        pathUpdateTimer = 0f;
        if (burgersInScene.Count == 0) {
            Burger[] burgers = FindObjectsByType<Burger>(FindObjectsSortMode.None);
            foreach (Burger b in burgers) burgersInScene.Add(b);
        }
    }

    public override void UpdateCharacter()
    {
        base.UpdateCharacter();
        pathUpdateTimer -= Time.deltaTime;
        if (pathUpdateTimer <= 0f) {
            pathUpdateTimer = PathUpdateInterval;
            UpdateZombieContext();
            TickBehaviorTree();
        }
    }

    void UpdateZombieContext()
    {
        GameObject[] zombieObjects = GetContextByTag("Zombie");
        zombiesInContext.Clear();
        foreach (GameObject zombieObj in zombieObjects) {
            Zombie zombie = zombieObj.GetComponent<Zombie>();
            if (zombie != null) zombiesInContext.Add(zombie);
        }
    }

    void TickBehaviorTree()
    {
        UpdateFleeFlag();
        if (TryCollectNearestBurger()) return;
        TrySeekFinish();
    }

    void UpdateFleeFlag()
    {
        isFleeing = false;
        foreach (Zombie zombie in zombiesInContext) {
            if (zombie == null) continue;
            if (Vector3.Distance(transform.position, zombie.transform.position) < SoftAvoidDistance) {
                isFleeing = true;
                break;
            }
        }
    }

    bool TrySeekFinish()
    {
        return TrySetPathToTarget(Grid.Instance.GetFinishTile(), zombiesInContext);
    }

    bool TryCollectNearestBurger()
    {
        Grid.Tile burgerTile = GetNearestActiveBurgerTile();
        if (burgerTile == null) return false;
        bool pathSet = TrySetPathToTarget(burgerTile, zombiesInContext);
        isFleeing = isFleeing || pathSet;
        return pathSet;
    }

    bool TrySetPathToTarget(Grid.Tile targetTile, List<Zombie> avoid)
    {
        if (myCurrentTile == null || targetTile == null) return false;
        List<Grid.Tile> newPath = FindPathAStar(myCurrentTile, targetTile, avoid);
        if (newPath == null || newPath.Count == 0) return false;
        currentPath = newPath;
        SetWalkBuffer(currentPath);
        return true;
    }

    List<Grid.Tile> FindPathAStar(Grid.Tile start, Grid.Tile goal, List<Zombie> zombiesToAvoid) {
        if (start == null || goal == null) return new List<Grid.Tile>();
        bool avoidZombies = zombiesToAvoid != null && zombiesToAvoid.Count > 0;
        float hardAvoidRadius = HardAvoidDistance;
        float softAvoidRadius = SoftAvoidDistance;
        List<AStarNode> openList = new();
        HashSet<Grid.Tile> closedSet = new();
        Dictionary<Grid.Tile, AStarNode> allNodes = new();
        AStarNode startNode = new(start, null, 0, GetHeuristic(start, goal));
        openList.Add(startNode);
        allNodes[start] = startNode;
        while (openList.Count > 0) {
            AStarNode currentNode = GetLowestFCostNode(openList);
            openList.Remove(currentNode);
            if (Grid.Instance.IsSameTile(currentNode.tile, goal)) return ReconstructPath(currentNode);
            closedSet.Add(currentNode.tile);
            List<Grid.Tile> neighbors = GetNeighbors(currentNode.tile);
            foreach (Grid.Tile neighborTile in neighbors) {
                if (closedSet.Contains(neighborTile) || neighborTile.occupied) continue;
                float tentativeGCost = currentNode.gCost + GetDistance(currentNode.tile, neighborTile);
                if (avoidZombies) {
                    Vector3 tilePos = Grid.Instance.WorldPos(neighborTile);
                    bool shouldSkipTile = false;
                    float totalPenalty = 0f;
                    foreach (Zombie zombie in zombiesToAvoid) {
                        if (zombie == null) continue;
                        float distToZombie = Vector3.Distance(tilePos, zombie.transform.position);
                        if (distToZombie < hardAvoidRadius) {
                            shouldSkipTile = true;
                            break;
                        }
                        if (distToZombie < softAvoidRadius) {
                            float penaltyFactor = 1f - Mathf.InverseLerp(hardAvoidRadius, softAvoidRadius, distToZombie);
                            totalPenalty += SoftPenalty * penaltyFactor;
                        }
                    }
                    
                    if (shouldSkipTile) continue;
                    tentativeGCost += totalPenalty;
                }
                AStarNode neighborNode;
                if (!allNodes.ContainsKey(neighborTile)) {
                    float hCost = GetHeuristic(neighborTile, goal);
                    neighborNode = new AStarNode(neighborTile, currentNode, tentativeGCost, hCost);
                    allNodes[neighborTile] = neighborNode;
                    openList.Add(neighborNode);
                } else {
                    neighborNode = allNodes[neighborTile];
                    if (tentativeGCost < neighborNode.gCost) {
                        neighborNode.gCost = tentativeGCost;
                        neighborNode.parent = currentNode;
                    }
                }
            }
        }
        return new List<Grid.Tile>();
    }

    Grid.Tile GetNearestActiveBurgerTile()
    {
        Grid.Tile nearest = null;
        float bestDist = float.MaxValue;
        foreach (Burger burger in burgersInScene) {
            if (burger == null || !burger.gameObject.activeInHierarchy) continue;
            Grid.Tile tile = Grid.Instance.GetClosest(burger.transform.position);
            if (tile == null || tile.occupied) continue;
            float dist = Vector3.Distance(transform.position, burger.transform.position);
            if (dist < bestDist) {
                bestDist = dist;
                nearest = tile;
            }
        }
        return nearest;
    }

    List<Grid.Tile> GetNeighbors(Grid.Tile tile) {
        List<Grid.Tile> neighbors = new();
        Vector2Int[] directions = new Vector2Int[] {
            new(1, 0),   // Right
            new(-1, 0),  // Left
            new(0, 1),   // Up
            new(0, -1),  // Down
            new(1, 1),   // Up-Right
            new(1, -1),  // Down-Right
            new(-1, 1),  // Up-Left
            new(-1, -1)  // Down-Left
        };
        foreach (Vector2Int dir in directions) {
            Grid.Tile neighbor = Grid.Instance.TryGetTile(new Vector2Int(tile.x + dir.x, tile.y + dir.y));
            if (neighbor == null || neighbor.occupied) continue;
            bool isDiagonal = Mathf.Abs(dir.x) + Mathf.Abs(dir.y) == 2;
            if (isDiagonal) {
                Grid.Tile sideA = Grid.Instance.TryGetTile(new Vector2Int(tile.x + dir.x, tile.y));
                Grid.Tile sideB = Grid.Instance.TryGetTile(new Vector2Int(tile.x, tile.y + dir.y));
                if ((sideA != null && sideA.occupied) || (sideB != null && sideB.occupied)) continue;
            }
            neighbors.Add(neighbor);
        }
        return neighbors;
    }

    AStarNode GetLowestFCostNode(List<AStarNode> openList) {
        AStarNode lowestNode = openList[0];
        foreach (AStarNode node in openList) {
            if (node.FCost < lowestNode.FCost || (node.FCost == lowestNode.FCost && node.hCost < lowestNode.hCost)) lowestNode = node;
        }
        return lowestNode;
    }

    float GetHeuristic(Grid.Tile a, Grid.Tile b) {
        int dx = Mathf.Abs(a.x - b.x);
        int dy = Mathf.Abs(a.y - b.y);
        return dx + dy + (Mathf.Sqrt(2) - 2) * Mathf.Min(dx, dy);
    }

    float GetDistance(Grid.Tile a, Grid.Tile b) {
        int dx = Mathf.Abs(a.x - b.x);
        int dy = Mathf.Abs(a.y - b.y);
        if (dx == 1 && dy == 1) return Mathf.Sqrt(2);
        return 1f;
    }

    List<Grid.Tile> ReconstructPath(AStarNode goalNode) {
        List<Grid.Tile> path = new();
        AStarNode currentNode = goalNode;
        while (currentNode != null) {
            path.Add(currentNode.tile);
            currentNode = currentNode.parent;
        }
        path.Reverse();
        return path;
    }

    private class AStarNode {
        public Grid.Tile tile;
        public AStarNode parent;
        public float gCost;
        public float hCost;
        public float FCost { get { return gCost + hCost; } }

        public AStarNode(Grid.Tile tile, AStarNode parent, float gCost, float hCost) {
            this.tile = tile;
            this.parent = parent;
            this.gCost = gCost;
            this.hCost = hCost;
        }
    }

    GameObject[] GetContextByTag(string aTag) {
        Collider[] context = Physics.OverlapSphere(transform.position, ContextRadius);
        List<GameObject> returnContext = new();
        foreach (Collider c in context) if (c.transform.CompareTag(aTag)) returnContext.Add(c.gameObject);
        return returnContext.ToArray();
    }

    private void OnDrawGizmos()
    {
        if (ShowPenaltyVisualization && zombiesInContext.Count > 0 && Grid.Instance != null && myCurrentTile != null) {
            float hardAvoidRadius = HardAvoidDistance;
            float softAvoidRadius = SoftAvoidDistance;
            int range = Mathf.CeilToInt(VisualizationRange);
            for (int x = -range; x <= range; x++) {
                for (int y = -range; y <= range; y++) {
                    Vector2Int tilePos = new Vector2Int(myCurrentTile.x + x, myCurrentTile.y + y);
                    Grid.Tile tile = Grid.Instance.TryGetTile(tilePos);
                    if (tile != null && !tile.occupied) {
                        Vector3 worldPos = Grid.Instance.WorldPos(tile);
                        float totalPenalty = 0f;
                        bool isBlocked = false;
                        foreach (Zombie zombie in zombiesInContext) {
                            if (zombie == null) continue;
                            float distToZombie = Vector3.Distance(worldPos, zombie.transform.position);
                            if (distToZombie < hardAvoidRadius) {
                                isBlocked = true;
                                break;
                            }
                            if (distToZombie < softAvoidRadius) {
                                float penaltyFactor = Mathf.InverseLerp(softAvoidRadius, hardAvoidRadius, distToZombie);
                                totalPenalty += SoftPenalty * penaltyFactor;
                            }
                        }
                        if (isBlocked) {
                            Gizmos.color = new Color(0.8f, 0f, 0f, 0.9f);
                            Gizmos.DrawCube(worldPos, Vector3.one * 0.8f);
                        } else if (totalPenalty > 0f) {
                            float normalizedPenalty = Mathf.Clamp01(totalPenalty / SoftPenalty);
                            Color penaltyColor = Color.Lerp(new Color(0f, 1f, 0f, 0.3f), new Color(1f, 0f, 0f, 0.6f), normalizedPenalty);
                            Gizmos.color = penaltyColor;
                            Gizmos.DrawCube(worldPos, Vector3.one * 0.6f);
#if UNITY_EDITOR
                            Handles.Label(worldPos + Vector3.up * 0.25f, totalPenalty.ToString("0.0"));
#endif
                        }
                    }
                }
            }
        }
        if (currentPath == null || currentPath.Count == 0) return;
        Color pathColor = isFleeing ? Color.red : Color.green;
        for (int i = 0; i < currentPath.Count; i++) {
            Vector3 position = Grid.Instance.WorldPos(currentPath[i]);
            Gizmos.color = pathColor;
            Gizmos.DrawWireSphere(position, 0.3f);
            if (i < currentPath.Count - 1) {
                Vector3 nextPosition = Grid.Instance.WorldPos(currentPath[i + 1]);
                Gizmos.DrawLine(position, nextPosition);
            }
        }
        foreach (Zombie zombie in zombiesInContext) {
            if (zombie != null) {
                Gizmos.color = new Color(1f, 0f, 0f, 0.2f);
                Gizmos.DrawWireSphere(zombie.transform.position, SoftAvoidDistance);
            }
        }
        Gizmos.color = new Color(0f, 0f, 1f, 0.2f);
        Gizmos.DrawWireSphere(transform.position, ContextRadius);
    }
}
