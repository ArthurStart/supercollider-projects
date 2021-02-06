(
var server, defaultGroup;

// var tstep = 0;
var stepdur = 1.0/30.0;

//--window setup
var width= 700, height= 700;

//--variables
var boids0 = [];
var boids1 = [];
var boids2 = [];
var boids3 = [];
var boids4 = [];

var speed0 = 3;
var makeBoid = { | pnt |  //pseudo class
    var dirn = pi.rand2;
    (
        \vel: speed0*Point(dirn.sin, dirn.cos),
        \pos: pnt,
        \acc: 0,
        \syn: Synth.basicNew(\flockSyn)
    )
};

// Interface:
~r_interact = 100;
~c_of_m_factor = 0.001;
~boundary = 100;
~speed_damping = 0.05;
~repulsion_dist = 30;
~vel_factor = 0.02;
~noise = 0.2;


server = Server(\nrt,
    options: ServerOptions.new
    .numOutputBusChannels_(2)
    .numInputBusChannels_(0)
    .maxNodes_(2048)
);
defaultGroup = Group.basicNew(server);



x = Score([
    [0.0, ['/d_recv',
        SynthDef(\flockSyn, { | freq, gate=1, px, py, speed, theta, acc, out=0 |
            var sig, env;
            freq = freq*theta.cos.linlin(-1, 1, 0.98, 1.02);

            env = EnvGen.kr(Env([0, 1, 0], [0.2, 10], releaseNode: 1), gate);

            sig = Pulse.ar(freq*[1, 2, 3, 4, 5]*LFNoise2.kr([0.3, 0.7, 2, 0.2, 3], 0.1*py, 1.0), width: speed.linlin(0, 2, 0.6, 0.9));
            // sig = SelectXFocus.ar(py.linlin(0, 1, 4, 0), sig, 3);
            sig = SelectXFocus.ar(SinOsc.kr((py*40)).range(4, 0), sig, 3);

            sig = DFM1.ar(sig, freq*(3 + (2*Lag2UD.kr(acc, 0.01, 1.0))), py.linlin(0, 1, 0.5, 0.9), 3) + BPF.ar(sig, speed.linlin(1, 2, freq, 10*freq));

            sig = sig + (0.4*(SinOsc.ar(freq*SinOsc.kr(0.1, 0, 0.04, 1.0), 0, 1.5*acc + 1) + SinOsc.ar(2*freq*SinOsc.kr(0.25, 0, 0.04, 1.0), 0, 1*((speed - 1)**2) + 2)));

            Out.ar(out, Pan2.ar(0.01*sig*env, px.linlin(0, 1, -1, 1)));
        }, #[1.0, 0.05, 0.05, 0.05, 0.05, 0.05, 0]).asBytes
    ]],

    [0.0, defaultGroup.newMsg]
]);





// Here begineth the NRT version of the animation func:


~n0 = 35;
~t0 = 0;

~n1 = 40;
~t1 = 60;

~n2 = 40;
~t2 = 105;

~n3 = 35;
~t3 = 120;

~n4 = 30;
~t4 = 130;

~notes0 = [32, 44, 51, 47, 56];
~notes1 = [31, 43, 51, 46, 55];

~tch1 = 170;
~tch2 = 180;

~tend = 190;

((~tend + 20)/stepdur).ceil.do { | tstep |
    var time = tstep*stepdur;
    time.postln;

    // time to make a new boid:

    if((time > ~t0) && (tstep%10 == 0) && (boids0.size < ~n0),
        {
            var b = makeBoid.value(Point(width.rand, height.rand));
            boids0 = boids0.add(b);
            x.add([time, b.syn.newMsg(g, [\freq, ~notes0[0].midicps])]);
        }
    );

    if((time > ~t1) && (tstep%5 == 0) && (boids1.size < ~n1),
        {
            var b = makeBoid.value(Point(width.rand, height.rand));
            boids1 = boids1.add(b);
            x.add([time, b.syn.newMsg(g, [\freq, ~notes0[1].midicps])]);
        }
    );

    if((time > ~t2) && (tstep%5 == 0) && (boids2.size < ~n2),
        {
            var b = makeBoid.value(Point(width.rand, height.rand));
            boids2 = boids2.add(b);
            x.add([time, b.syn.newMsg(g, [\freq, ~notes0[2].midicps])]);
        }
    );

    if((time > ~t3) && (tstep%5 == 0) && (boids3.size < ~n3),
        {
            var b = makeBoid.value(Point(width.rand, height.rand));
            boids3 = boids3.add(b);
            x.add([time, b.syn.newMsg(g, [\freq, ~notes0[3].midicps])]);
        }
    );

    if((time > ~t4) && (tstep%5 == 0) && (boids4.size < ~n4),
        {
            var b = makeBoid.value(Point(width.rand, height.rand));
            boids4 = boids4.add(b);
            x.add([time, b.syn.newMsg(g, [\freq, ~notes0[4].midicps])]);
        }
    );

    [boids0, boids1, boids2, boids3, boids4].do{ | boids, bgroup |
        boids.do{ | b |  // update each boid
            var vel = b.vel; // unit velocity
            var n_nbs = 0;
            var c_of_m = Point(0, 0);
            var v_perceived = Point(0, 0);
            var wall = 0;

            vel = vel.asArray.min(20).asPoint;

            // Walls:
            if(b.pos.x < ~boundary, {
                vel = vel + [0.5*((~boundary/b.pos.x) - 1).min(width/2), 0];
                wall = 1;
            });
            if(b.pos.x > (width - ~boundary), {
                vel = vel + [0.5*(1 - (~boundary/(width - b.pos.x))).min(width/2), 0];
                wall = 1;
            });
            if(b.pos.y < ~boundary, {
                vel = vel + [0, 0.5*((~boundary/b.pos.y) - 1).min(height/2)];
                wall = 1;
            });
            if(b.pos.y > (height - ~boundary), {
                vel = vel + [0, 0.5*(1 - (~boundary/(height - b.pos.y))).min(height/2)];
                wall = 1;
            });

            // Nearby boids:
            boids.do{ | nb |
                var dist = b.pos.dist(nb.pos);
                if( (dist < ~r_interact) && (dist > 0), // Then nb is a NearBoid
                    {
                        n_nbs = n_nbs + 1;

                        // Rule 1. Centre of mass.
                        c_of_m = c_of_m + nb.pos;

                        // Rule 2. Don't touch!
                        if(dist < ~repulsion_dist, {
                            vel = vel + (0.5*((b.pos - nb.pos)/dist)*(((~repulsion_dist/dist) - 1)**3) );
                        });


                        // Rule 3. Match velocities.
                        v_perceived = v_perceived + nb.vel;
                    }
                )
            };

            vel = vel.asArray.min(5).asPoint;


            if( n_nbs > 0,
                {
                    c_of_m = c_of_m/n_nbs;
                    vel = vel + ((c_of_m - b.pos)*~c_of_m_factor);

                    v_perceived = v_perceived/n_nbs;
                    vel = vel + (v_perceived*~vel_factor);
                }
            );

            vel = vel.asArray.min(5).asPoint;

            // Add noise:
            vel = vel + (~noise*[1.0.sum3rand, 1.0.sum3rand]);

            vel = vel * ((speed0 - vel.rho)*~speed_damping + 1);


            vel = vel.asArray.min(5).asPoint;


            b.pos = b.pos + vel;  //move the boid
            b.acc = (b.vel - vel).rho;
            b.vel = vel;


            if((time > ~tch1) && (time < ~tch2) && (wall == 1), {
                x.add([
                    time, [
                        '/n_set', b.syn.nodeID,
                        \freq, ~notes1[bgroup].midicps
                    ]
                ]);
            });

            if((time > ~tch2) && (wall == 1), {
                x.add([
                    time, [
                        '/n_set', b.syn.nodeID,
                        \freq, ~notes0[bgroup].midicps
                    ]
                ]);
            });

            if((time > ~tend) && (wall == 1), {
                x.add([
                    time, [
                        '/n_set', b.syn.nodeID,
                        \gate, 0
                    ]
                ]);
            });

            x.add([
                time, [
                    '/n_set', b.syn.nodeID,
                    \px, b.pos.x/width,
                    \py, b.pos.y/height,
                    \speed, b.vel.rho/speed0,
                    \theta, b.vel.theta,
                    \acc, b.acc
                ]
            ]);

        }
    };
};

x.recordNRT(
outputFilePath: "/Users/arthurstart/Music/SuperCollider Recordings/nrt.aiff".standardizePath,
sampleRate: 44100,
headerFormat: "AIFF",
sampleFormat: "int24",
options: server.options,
duration: x.endTime + 15
);

server.remove;
)
