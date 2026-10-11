class InsertionPlanner:
    """Insertion-order planner for fixed 3D-BPP targets.

    The packing side owns ``final_target`` and orientation.  This class never
    moves those targets.  It only:
      * derives pairwise route-blocking precedence constraints,
      * adds support-before-upper-item constraints,
      * topologically orders the fixed packing plan,
      * chooses an executable visible item at runtime,
      * applies a temporary Z lift for shelf insertion only.

    Edge convention:
        A -> B  means  A must be inserted before B.
    """

    def __init__(self, path_clear_fn, runtime_path_clear_fn=None, runtime_support_fn=None,
                 static_path_clear_fn=None, shelf_lift=0.021):
        # path_clear_fn: pairwise packed-item route test used only for precedence.
        # runtime_path_clear_fn: full runtime test (packed items + static shelves).
        # static_path_clear_fn: fixed-shelf-only test.  A failure here is
        # permanent for the fixed BPP target, unlike a movable-item blockage.
        self.path_clear_fn = path_clear_fn
        self.runtime_path_clear_fn = runtime_path_clear_fn or path_clear_fn
        self.runtime_support_fn = runtime_support_fn
        self.static_path_clear_fn = static_path_clear_fn
        self.shelf_lift = float(shelf_lift)
        self.last_runtime_diagnostics = {}

    def execution_target(self, plan):
        target = tuple(float(v) for v in plan['final_target'])
        lift = 0.0
        if plan.get('support_kind') in ('small-shelf', 'main-shelf'):
            lift = self.shelf_lift
            target = (target[0], target[1], target[2] + lift)
        return target, lift

    def build_precedence(self, plans, containers):
        """Build route/support precedence graph and a deterministic topo order.

        v5 cycle policy:
          * support edges are mandatory and are never removed;
          * route edges are added only when they do not create a cycle;
          * route edges that reverse the BPP rank are considered first, because
            those are exactly the constraints that must change the packing order
            to keep an insertion corridor open;
          * a route edge that contradicts an existing support chain is dropped.

        This keeps the 3D-BPP targets fixed while preventing the old behaviour
        where an unresolved cycle was appended in plan-rank order and could
        silently violate a known route constraint (e.g. 20 -> 9).
        """
        plans = list(plans)
        by_id = {int(p['item_id']): p for p in plans}
        edges = {iid: set() for iid in by_id}
        incoming = {iid: set() for iid in by_id}
        reasons = {}
        dropped_route_edges = []

        def add_edge(a, b, reason):
            a, b = int(a), int(b)
            if a == b or a not in edges or b not in edges:
                return False
            if b not in edges[a]:
                edges[a].add(b)
                incoming[b].add(a)
            reasons.setdefault((a, b), []).append(str(reason))
            return True

        def has_path(src, dst):
            src, dst = int(src), int(dst)
            if src == dst:
                return True
            seen = set()
            stack = [src]
            while stack:
                cur = stack.pop()
                if cur in seen:
                    continue
                seen.add(cur)
                for nxt in edges.get(cur, ()):
                    if nxt == dst:
                        return True
                    if nxt not in seen:
                        stack.append(nxt)
            return False

        # Physical support is authoritative: supporter must exist before upper.
        for p in plans:
            support_id = p.get('support_item_id')
            if support_id is not None and int(support_id) in by_id:
                add_edge(int(support_id), int(p['item_id']), 'support')

        rank = {int(p['item_id']): int(p.get('plan_rank', 10**9)) for p in plans}

        # First discover all pairwise route constraints without mutating graph.
        route_candidates = []
        seen_route = set()
        for moving in plans:
            mid = int(moving['item_id'])
            cidx = int(moving['container_idx'])
            c = containers[cidx]
            exec_target, _ = self.execution_target(moving)
            support_id = moving.get('support_item_id')
            support_id = None if support_id is None else int(support_id)
            for blocker in plans:
                bid = int(blocker['item_id'])
                if bid == mid or int(blocker['container_idx']) != cidx:
                    continue
                # The intended physical supporter is expected to be present.
                if support_id is not None and bid == support_id:
                    continue
                packed_record = blocker.get('packed_record')
                if packed_record is None:
                    continue
                clear = bool(self.path_clear_fn(
                    c,
                    moving['item'],
                    exec_target,
                    [packed_record],
                ))
                if not clear and (mid, bid) not in seen_route:
                    seen_route.add((mid, bid))
                    route_candidates.append((mid, bid))

        # A route edge that reverses BPP rank carries information the BPP order
        # itself does not provide, so preserve those first. Larger reversals are
        # considered first, then use deterministic ids.
        route_candidates.sort(
            key=lambda ab: (
                0 if rank[ab[0]] > rank[ab[1]] else 1,
                -abs(rank[ab[0]] - rank[ab[1]]),
                rank[ab[0]],
                rank[ab[1]],
                ab[0],
                ab[1],
            )
        )

        for a, b in route_candidates:
            # If b already reaches a through mandatory support / accepted route
            # dependencies, adding a->b would create a cycle.  Drop only this
            # route approximation; never remove support edges.
            if has_path(b, a):
                dropped_route_edges.append((a, b, 'would-create-cycle'))
                continue
            add_edge(a, b, 'route')

        # Stable Kahn topological sort. With route-cycle pruning, only an actual
        # support cycle (which should not be produced by a valid BPP stack) can
        # remain cyclic here.
        indeg = {iid: len(preds) for iid, preds in incoming.items()}
        ready = sorted((iid for iid, d in indeg.items() if d == 0), key=lambda x: (rank[x], x))
        order = []
        while ready:
            iid = ready.pop(0)
            order.append(iid)
            for nxt in sorted(edges[iid], key=lambda x: (rank[x], x)):
                indeg[nxt] -= 1
                if indeg[nxt] == 0:
                    ready.append(nxt)
                    ready.sort(key=lambda x: (rank[x], x))

        order_set = set(order)
        cyclic = [iid for iid in by_id if iid not in order_set]
        if cyclic:
            cyclic.sort(key=lambda x: (rank[x], x))
            order.extend(cyclic)

        return {
            'order': order,
            'edges': edges,
            'incoming': incoming,
            'reasons': reasons,
            'cyclic': cyclic,
            'dropped_route_edges': dropped_route_edges,
        }

    def choose_runtime(self, visible_plans, containers, packed_by_container,
                       graph=None, placed_ids=None):
        """Choose a currently executable plan without changing its final target.

        Fixed-shelf blockage is treated specially.  A plan whose fixed target
        cannot be reached because of a shelf is never executed as a last resort,
        and route-only outgoing edges from that permanently unreachable plan do
        not hold back other items.  Physical support edges remain authoritative.
        """
        placed_ids = set(int(x) for x in (placed_ids or set()))
        order_rank = {}
        incoming = {}
        reasons = {}
        if graph is not None:
            order_rank = {iid: i for i, iid in enumerate(graph.get('order', []))}
            incoming = graph.get('incoming', {})
            reasons = graph.get('reasons', {})

        ordered = sorted(
            visible_plans,
            key=lambda p: (
                order_rank.get(int(p['item_id']), 10**9),
                int(p.get('plan_rank', 10**9)),
                int(p['item_id']),
            ),
        )

        # Classify permanent fixed-shelf failures first.  This is independent of
        # packed-item order, so retrying or reordering cannot make them reachable.
        static_blocked = set()
        static_meta = {}
        if self.static_path_clear_fn is not None:
            for p in ordered:
                iid = int(p['item_id'])
                cidx = int(p['container_idx'])
                execution_target, _ = self.execution_target(p)
                clear = bool(self.static_path_clear_fn(
                    containers[cidx], p['item'], execution_target
                ))
                if not clear:
                    static_blocked.add(iid)
                    static_meta[iid] = {
                        'reason': 'STATIC_SHELF_BLOCKED',
                        'execution_target': execution_target,
                    }

        def route_only_edge(pred, cur):
            rs = [str(x) for x in reasons.get((int(pred), int(cur)), [])]
            # If support is among the reasons, the dependency cannot be waived.
            return bool(rs) and 'support' not in rs and all(r == 'route' for r in rs)

        blocked_meta = []
        for p in ordered:
            iid = int(p['item_id'])
            if iid in static_blocked:
                blocked_meta.append((iid, 'static-shelf', static_meta[iid]))
                continue

            unmet = []
            ignored_static_route_preds = []
            for pred in sorted(int(x) for x in incoming.get(iid, set())):
                if pred in placed_ids:
                    continue
                if pred in static_blocked and route_only_edge(pred, iid):
                    ignored_static_route_preds.append(pred)
                    continue
                unmet.append(pred)
            if unmet:
                blocked_meta.append((iid, 'precedence', unmet))
                continue

            cidx = int(p['container_idx'])
            c = containers[cidx]
            packed = packed_by_container.get(cidx, [])

            support_meta = {'ok': True, 'reason': 'not-checked'}
            if self.runtime_support_fn is not None:
                support_meta = self.runtime_support_fn(p, c, packed)
                if not bool(support_meta.get('ok', False)):
                    blocked_meta.append((iid, 'runtime-support', support_meta))
                    continue

            execution_target, lift = self.execution_target(p)
            runtime_packed = packed
            support_id = p.get('support_item_id')
            if str(p.get('support_kind', '')) == 'box' and support_id is not None:
                sid = int(support_id)
                runtime_packed = [
                    obj for obj in packed
                    if obj.get('index') is None or int(obj['index']) != sid
                ]
            ok = bool(self.runtime_path_clear_fn(c, p['item'], execution_target, runtime_packed))
            if ok:
                self.last_runtime_diagnostics = {
                    'static_blocked_ids': sorted(static_blocked),
                    'static_meta': static_meta,
                    'blocked_meta': blocked_meta,
                    'selected': iid,
                    'ignored_static_route_predecessors': ignored_static_route_preds,
                }
                return p, execution_target, {
                    'path_ok': True,
                    'shelf_lift': lift,
                    'unmet_predecessors': [],
                    'blocked_meta': blocked_meta,
                    'support_meta': support_meta,
                    'ignored_static_route_predecessors': ignored_static_route_preds,
                }
            blocked_meta.append((iid, 'runtime-path-dynamic-or-conservative', []))

        self.last_runtime_diagnostics = {
            'static_blocked_ids': sorted(static_blocked),
            'static_meta': static_meta,
            'blocked_meta': blocked_meta,
            'selected': None,
        }
        return None
