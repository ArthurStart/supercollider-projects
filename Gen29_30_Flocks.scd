StageLimiter.activate;
s.options.maxNodes = 2048;
s.options.numInputBusChannels = 0;
s.options.hardwareBufferSize = 2048;

(
s.latency= 0.05;
s.waitForBoot{

    //--window setup
    var width= 700, height= 700;
    var w= Window("Flocking Boids", Rect(99, 99, width, height), false);
    var u= UserView(w, Rect(0, 0, width, height));

    //--variables
    var synths = ();  //keep track of synths objects
    var boids = [];
    var speed0 = 3;
    var makeBoid = { | pnt |  //pseudo class
        var dirn = pi.rand2;
        (
            \vel: speed0*Point(dirn.sin, dirn.cos),
            \pos: pnt,
            \acc: 0,
            \syn: Synth(\flockSyn, [\freq, 50])
        )
    };

    /*SynthDef(\av, {|freq= 400, fm= 1, beat= 1, amp= 0, pan= 0, gate= 1|
        var e= EnvGen.ar(Env.asr(0.01, 1, 0.02), gate, doneAction:2);
        var z= SinOsc.ar(freq*SinOsc.ar(0, SinOsc.ar(fm, 0, 2pi), beat), 0, amp);
        Out.ar(0, Pan2.ar(z, pan, e));
    }, #[0.05, 0.05, 0.05, 0.05, 0.05]).add;*/

    SynthDef(\flockSyn, { | freq, px, py, speed, theta, acc, out=0 |
        var sig;
        freq = freq*theta.cos.linlin(-1, 1, 0.98, 1.02);

        sig = Pulse.ar(freq*[1, 2, 3, 4, 5]*LFNoise2.kr([0.3, 0.7, 2, 0.2, 3], 0.1*py, 1.0), width: speed.linlin(0, 2, 0.6, 0.9));
        // sig = SelectXFocus.ar(py.linlin(0, 1, 4, 0), sig, 3);
        sig = SelectXFocus.ar(SinOsc.kr((py*40)).range(4, 0), sig, 3);

        sig = DFM1.ar(sig, freq*(3 + (2*Lag2UD.kr(acc, 0.01, 1.0))), py.linlin(0, 1, 0.5, 0.9), 3) + BPF.ar(sig, speed.linlin(1, 2, freq, 10*freq));

        sig = sig + SinOsc.ar(freq*SinOsc.kr(0.1, 0, 0.04, 1.0), 0, 1.5*acc + 1) + SinOsc.ar(2*freq*SinOsc.kr(0.25, 0, 0.04, 1.0), 0, 1*((speed - 1)**2) + 2);

        Out.ar(out, Pan2.ar(0.03*sig, px.linlin(0, 1, -1, 1)));
    }, #[0, 0.05, 0.05, 0.05, 0.05, 0.05, 0]).add;

    s.sync;

    // make boid when mouse clicked
    u.mouseDownAction_{ | v, x, y |
        boids = boids.add(makeBoid.value(Point(x, y)));
    };

    // Interface:
    ~r_interact = 100;
    ~c_of_m_factor = 0.001;
    ~boundary = 100;
    ~speed_damping = 0.05;
    ~repulsion_dist = 30;
    ~vel_factor = 0.02;
    ~noise = 0.2;

    //--main loop
    u.drawFunc= {

        boids.do{ | b |  // update each boid
            var vel = b.vel; // unit velocity
            var n_nbs = 0;
            var c_of_m = Point(0, 0);
            var v_perceived = Point(0, 0);

            // Walls:
            if(b.pos.x < ~boundary, {
                vel = vel + [0.5*((~boundary/b.pos.x) - 1), 0];
            });
            if(b.pos.x > (width - ~boundary), {
                vel = vel + [0.5*(1 - (~boundary/(width - b.pos.x))), 0];
            });
            if(b.pos.y < ~boundary, {
                vel = vel + [0, 0.5*((~boundary/b.pos.y) - 1)];
            });
            if(b.pos.y > (height - ~boundary), {
                vel = vel + [0, 0.5*(1 - (~boundary/(height - b.pos.y)))];
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
                            // vel = vel + ((1 - (dist/~repulsion_dist))*((b.pos - nb.pos)));
                            // vel = vel + ((b.pos - nb.pos)/dist);
                        });

                        // Rule 3. Match velocities.
                        v_perceived = v_perceived + nb.vel;
                    }
                )
            };

            if( n_nbs > 0,
                {
                    c_of_m = c_of_m/n_nbs;
                    vel = vel + ((c_of_m - b.pos)*~c_of_m_factor);

                    v_perceived = v_perceived/n_nbs;
                    vel = vel + (v_perceived*~vel_factor);
                }
            );


            // Add noise:
            vel = vel + (~noise*[1.0.sum3rand, 1.0.sum3rand]);

            vel = vel * ((speed0 - vel.rho)*~speed_damping + 1);

            b.pos = b.pos + vel;  //move the boid
            b.acc = (b.vel - vel).rho;
            b.vel = vel;

            /*p.syn.set(  //system maps to sound
                \freq, p.pos.y.linexp(0, height, 2000, 200),
                \amp, p.pos.y.linlin(0, height, p.mas, 0)*(1-(p.age/~dead))*0.1,
                \fm, p.mas*p.pos.x,
                \beat, p.vel.asComplex.magnitude,  //ball velocity mapped to beat
                \pan, p.pos.x.linlin(0, width, -1, 1)
            );*/

            b.syn.set(
                \px, b.pos.x/width,
                \py, b.pos.y/height,
                \speed, b.vel.rho/speed0,
                \theta, b.vel.theta,
                \acc, b.acc
            );

            Pen.fillColor = Color.grey((b.acc*0.5) + 0.2, 1);
            Pen.fillOval(Rect.aboutPoint(b.pos, 5, 5));
        };
    };

    //--window management
    u.clearOnRefresh= true;
    u.background= Color.black;
    // w.onClose= {parts.do{|p| p.syn.free}};
    w.front;
    u.animate= true;
    CmdPeriod.doOnce({if(w.isClosed.not, {w.close})});
};
)

//change these while the program is running
~mass= 4;
~grav= Point(0.1, 0.1);
~grav= Point(-0.1, -0.1);
~damp= 0.9;



Point(3, 4).theta