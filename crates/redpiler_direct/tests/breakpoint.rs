use mchprs_redpiler::backend::{GraphBackend, GraphContext};
use mchprs_redpiler::circuit::{Circuit, NodeId};
use mchprs_redpiler::graph::{Graph, Node, NodeKind, NodeState, QueuedTick};
use mchprs_redpiler_direct::backend::DirectBackend;
use mchprs_redstone::tick_queue::{PendingTick, TickPriority};
use mchprs_simulation::readback::Readback;
use mchprs_simulation::ticks::{Ran, Stop};
use mchprs_simulation::watch::{Bit, Breakpoint, BreakpointId, Condition, Hit, Watch};

#[test]
fn breakpoint_stops_after_every_due_node_has_ticked() {
    let mut graph = Graph::default();
    for (order, delay) in [1, 1, 2].into_iter().enumerate() {
        graph.add_node(Node {
            kind: if delay == 2 {
                NodeKind::Lamp
            } else {
                NodeKind::Torch
            },
            blocks: Default::default(),
            names: Vec::new(),
            state: NodeState::from_powered(delay == 2),
            pending_tick: Some(QueuedTick {
                tick: PendingTick {
                    delay,
                    priority: TickPriority::Normal,
                },
                order,
            }),
            port: None,
            observed: true,
        });
    }
    let circuit = Circuit::new(graph).unwrap();
    let mut engine = DirectBackend
        .build(&circuit, &Default::default(), &mut GraphContext::silent())
        .unwrap();
    let breakpoint = BreakpointId(7);
    engine
        .watch(Watch {
            breakpoints: vec![Breakpoint {
                id: breakpoint,
                bits: vec![Bit::At(NodeId::new(0))].into_boxed_slice(),
                condition: Condition::Becomes(1),
            }],
        })
        .unwrap();
    assert_eq!(
        engine.run(10).unwrap(),
        Ran {
            ticks: 1,
            stop: Some(Stop::Breakpoint(Hit {
                breakpoint,
                tick: 1,
                value: 1
            })),
        }
    );
    let Readback::Complete(snapshot) = engine.read().unwrap() else {
        panic!("direct provides complete readback");
    };
    assert_eq!(
        snapshot
            .states
            .iter()
            .map(|state| state.is_powered())
            .collect::<Vec<_>>(),
        [true, true, true]
    );
    assert_eq!(
        snapshot.pending_ticks,
        [(
            NodeId::new(2),
            PendingTick {
                delay: 1,
                priority: TickPriority::Normal
            }
        )]
    );
}
