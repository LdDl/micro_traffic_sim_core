# merge-blocked

Demonstrates the "keep rolling along a jammed lane" fallback in
`find_alternate_intention` and the confusion A*-skip, on the real stage network
(`network.json` - 676 cells exported from my local toy client application: forward/left/right
links, WGS84 coordinates, zones, speed limits).

Vehicle 1 drives 666 -> 667 -> 668 (no right neighbor) toward destination 621,
reachable only via the left merges 666->605, 667->606, 668->619. Past 668 lies
the point of no return (681 -> 683 death zone). The left lane is jammed with
parked vehicles.

```bash
cargo run --example merge-blocked
```

| Scenario | Parked | Expected outcome |
|---|---|---|
| A: gap at the last merge | 605, 606 | rolls along the jam, merges at 619 -> COMPLETED |
| B: fully jammed | 605, 606, 619 | rolls past the last merge, confusion drives it to the 683 death zone -> LOST, the lane stays free (accepted risk) |

A DEADLOCK verdict (standing next to the jam forever) is a regression: the
forward fallback in `find_alternate_intention` stopped working.

Once a vehicle is confused, its destination is unreachable from every cell it
can ever reach (reachability is monotone along directed edges), so per-tick A*
is skipped for it entirely - a failed full A* is the most expensive kind.
