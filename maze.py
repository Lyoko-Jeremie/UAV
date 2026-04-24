from collections import deque
from typing import List, Optional, Tuple

# 定义坐标点类型提示，使用元组 (row, col) 使代码更具 Python 风格
Point = Tuple[int, int]


def find_shortest_path(maze: List[List[int]], start: Point, end: Point) -> Optional[List[Point]]:
    """
    使用广度优先搜索 (BFS) 在迷宫中寻找最短路径
    :param maze: 二维矩阵，0 表示互通/可走，1 表示障碍物
    :param start: 起点坐标 (row, col)
    :param end: 终点坐标 (row, col)
    :return: 返回从起点到终点的最短路径列表；如果无解则返回 None
    """
    rows = len(maze)
    if rows == 0:
        return None
    cols = len(maze[0])

    start_r, start_c = start
    end_r, end_c = end

    # 边界检查与合法性校验：起点或终点在网格外部，或者本身就是障碍物
    if (
            start_r < 0 or start_r >= rows or start_c < 0 or start_c >= cols or
            end_r < 0 or end_r >= rows or end_c < 0 or end_c >= cols or
            maze[start_r][start_c] != 0 or maze[end_r][end_c] != 0
    ):
        return None

    # 队列用于 BFS 遍历，使用 deque 提供 O(1) 的 popleft 性能
    queue = deque([start])

    # 记录已经访问过的节点，避免走回头路和死循环
    visited = [[False] * cols for _ in range(rows)]
    visited[start_r][start_c] = True

    # 记录路径：parent 记录每个节点是从哪个相邻节点走过来的
    # 使用二维数组存放父节点引用（存储父节点的坐标元组）
    parent: List[List[Optional[Point]]] = [[None] * cols for _ in range(rows)]

    # 定义四个移动方向：上、下、左、右
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    is_reached = False

    # 开始 BFS
    while queue:
        # 出队当前节点
        current_r, current_c = queue.popleft()

        # 检查是否到达终点
        if current_r == end_r and current_c == end_c:
            is_reached = True
            break

        # 探索相邻的四个方向
        for dr, dc in directions:
            next_r = current_r + dr
            next_c = current_c + dc

            # 判断相邻节点是否在矩阵范围内，是否可通行，且尚未被访问过
            # Python 允许链式比较，例如 0 <= next_r < rows
            if (
                    0 <= next_r < rows and
                    0 <= next_c < cols and
                    maze[next_r][next_c] == 0 and
                    not visited[next_r][next_c]
            ):
                # 标记为已访问，并加入队列
                visited[next_r][next_c] = True
                queue.append((next_r, next_c))

                # 记录是从 current 节点走到 next 节点的
                parent[next_r][next_c] = (current_r, current_c)

    # 如果队列清空依然没有到达终点，说明没有连通的路径
    if not is_reached:
        return None

    # 回溯生成最短路径
    path: List[Point] = []
    current_path_node = end

    while current_path_node is not None:
        path.append(current_path_node)
        curr_r, curr_c = current_path_node
        current_path_node = parent[curr_r][curr_c]

    # 因为我们是从终点往回追溯到起点的，所以需要反转数组
    path.reverse()
    return path
