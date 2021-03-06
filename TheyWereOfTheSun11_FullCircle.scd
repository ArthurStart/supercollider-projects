// Start Jack with JackPilot. Make sure 16 virtual channels are configured. Do these things. Run Reaper and set to JackRouter. Make connections with JackPilot.

Server.supernova;
Server.scsynth;

s.options.device_("JackRouter");
s.options.memSize = 32768*8; // in kB
s.options.sampleRate = 48000;
s.options.numInputBusChannels = 0;
s.options.numOutputBusChannels = 32;
// s.options.numOutputBusChannels = 2;
s.boot;





(    /* SETUP MAIN */

/* Define the solar system */
~planet_specifications = Dictionary.newFrom([
    \b, (bus_index: 0, radius: 0.074, planet_radius: 1.31, period: 7.008151, phi: 0),
    \c, (bus_index: 1, radius: 0.089, planet_radius: 1.18, period: 8.719375, phi: 0),
    \i, (bus_index: 2, radius: 0.107, planet_radius: 1.32, period: 14.44912, phi: 0),
    \d, (bus_index: 3, radius: 0.32, planet_radius: 2.88, period: 59.73667, phi: 0),
    \e, (bus_index: 4, radius: 0.43, planet_radius: 2.67, period: 91.93913, phi: 0),
    \f, (bus_index: 5, radius: 0.48, planet_radius: 2.89, period: 124.9144, phi: 0),
    \g, (bus_index: 6, radius: 0.71, planet_radius: 8.13, period: 210.60697, phi: 0),
    \h, (bus_index: 7, radius: 1.01, planet_radius: 11.32, period: 331.60059, phi: 0),
]);


/* Define Buses */
~numPlanets = ~planet_specifications.size;
~planetsControlBusses = Array.fill(~numPlanets, {Bus.control(s, 4)});
~rateControlBus = Bus.control(s, 1);
~planetsGroup = Group.new(s);



// Ambisonics:
~order = 5;
~hoaNumChannels = (~order + 1).pow(2);
// ~hoaBus = Bus.audio(s, ~hoaNumChannels);
// ~recBus = Bus.audio(s, 2);
HOABinaural.freeBinauralIRs;
HOABinaural.loadbinauralIRs;


/* Define generic planet */
~makePlanet = { | name, synth_func, planet_spec, observer_spec |
    var p_index = planet_spec[\bus_index];
    SynthDef(name,
        {
            arg doppler_strength=400, playback_rate=2, vio=0.1, dark=0.5, amp=0.8;

            var sig, hoaSig, binauralSig, sigbus, amp_fac, sr;
            var freq_multiplier, omega, theta_dash, r_p, r_o, theta_pan, disp, d_squared, d_dot, d_min;

            // playback_rate = XLine.kr(75, 2, 20*60);

            sr = SampleRate.ir;
            r_p = 100*planet_spec[\radius];
            r_o = 100*observer_spec[\radius];

            // Planetary dynamics (assuming circular orbits):
            omega = 2pi*((1/observer_spec[\period]) - (1/planet_spec[\period]));
            theta_dash = Phasor.ar(rate: playback_rate*omega/sr, start: -pi, end: pi);


            d_squared = r_p.squared + r_o.squared - (2*r_p*r_o*cos(theta_dash));
            disp = d_squared.sqrt;
            theta_pan = theta_dash.sign * ( (d_squared + r_o.squared - r_p.squared) / (2*disp*r_o)).acos;

            d_dot = (r_p*r_o/disp)*sin(theta_dash)*omega;
            d_min = (r_p - r_o).abs;

            freq_multiplier = (1 - (d_dot/doppler_strength)); // Doppler Shift.

            // theta_pan.scope;

            // amp = LinExp.kr(amp, 0, 1, 1, 2) - 1;
            sig = SynthDef.wrap(synth_func, prependArgs: [amp, vio, dark, freq_multiplier]);
			sig = HPF.ar(sig, 10);
            sig = CompanderD.ar(sig, 0.12, 1, 0.01);

            amp_fac = d_min/disp;

            // BASIC:
            /*sig = Pan2.ar(sig*amp_fac, theta_pan.sin);
            sig = FreeVerb2.ar(sig[0], sig[1], room: size*disp);*/

            // OR BINAURAL:
            hoaSig = HBHOAEncoder.ar(~order, sig*amp_fac, theta_pan, 1);
            binauralSig = HOABinaural.ar(~order, hoaSig);

            Out.ar(2*planet_spec[\bus_index], binauralSig);
    }).add;

};


// Synths for each individual planet are composed here:
~makePlanet.value(\b_from_e,
    { | amp, vio, dark, freq_multiplier, freq, wait, hc_amp=0.1 |
        var sig;

        freq = Lag.kr(freq, wait)*freq_multiplier*[1, 2, 3, 4, 6];
        sig = FreeVerb.ar(
            Mix(
                SinOsc.ar(freq*[1+(0.03*vio), 2, 1-(0.03*vio)])
                + Saw.ar(freq*[1-(0.01*vio), Integrator.kr(Dust.kr(50), 0.99*vio)], mul: vio)
            ),
            mix: 0.5,
            room: 5,
            damp: dark
        );
        // sig = BLowShelf.ar(sig, 300, 5, 9);
        sig = LPF.ar(sig, 1000-(900*dark));
        sig = HPF.ar(sig, 100, mul: amp*hc_amp);
        sig = (0.7*LPF.ar(sig, 250)) + (0.3*sig);
    },
    planet_spec: ~planet_specifications[\b],
    observer_spec: ~planet_specifications[\e]
);


~makePlanet.value(\c_from_e,
    { | amp, vio, dark, freq_multiplier, freq, wait, hc_amp=0.05 |
        var sig;

        freq = Lag.kr(freq, wait)*freq_multiplier;
        sig = LFTri.ar(freq*(1..6),
            500,
            mul: ([1] ++ (vio*[1, 0.8, 0.6]) ++ (vio.squared*[1, 0.8]))
        );
        sig = Mix.ar(BPF.ar(sig, (0.5*freq)+(500*vio), 0.5+(2*dark)));
        sig = sig*LinExp.kr(vio**2, 1, 0, 1, 30);
        sig = HPF.ar(sig, 300);
        sig = FreeVerb.ar(sig, 0.6*vio, mul: amp*hc_amp);
    },
    planet_spec: ~planet_specifications[\c],
    observer_spec: ~planet_specifications[\e]
);

~makePlanet.value(\i_from_e,
    { | amp, vio, dark, freq_multiplier, freq, wait, hc_amp=0.2 |
        var sig;

        freq = Lag.kr(freq, wait)*freq_multiplier;

        sig = (0.5*SinOsc.ar(freq).softclip) + (vio*(
            LFNoise1.ar(freq, mul: 1-dark)
            + Mix(SinOsc.ar(freq*[0.5, 0.25, 0.125], mul: 1-dark))
            + Mix(0.1*SinOsc.ar(freq*[2, 4, 6, 8], mul: dark*[0.8, 0.6, 0.5, 0.4])
            + LFNoise0.ar(freq*[6, 8], mul: 0.05*dark))
        ));
        sig = amp*hc_amp*sig;
    },
    planet_spec: ~planet_specifications[\i],
    observer_spec: ~planet_specifications[\e]
);

~makePlanet.value(\d_from_e,
    { | amp, vio, dark, freq_multiplier, freq, wait, hc_amp=2 |
        var sig;

        freq = Lag.kr(freq, wait)*freq_multiplier;

        sig = WhiteNoise.ar(mul: dark) + (0.9*GrayNoise.ar(mul: 1 - dark));
        sig = Mix(BPF.ar(
            sig,
            (freq)*[1, 2, 3, 4, 5, 6, 7],
            mul: SinOsc.kr(160*[0.53569237465, 0.93753, 0.3234, 0.456783, 0.24856, 0.355764334, 0.0223524]).range(1-vio, 1)*[1, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4],
            rq: 0.1
        ));

        sig = RHPF.ar(LPF.ar(sig, 150+(1000*dark)), 20 + (100*dark), mul: amp*hc_amp);
    },
    planet_spec: ~planet_specifications[\d],
    observer_spec: ~planet_specifications[\e]
);

~makePlanet.value(\f_from_e,
    { | amp, vio, dark, freq_multiplier, freq, wait, hc_amp=0.2 |
        var sig;

        freq = Lag.kr(freq, wait)*freq_multiplier;

        sig = Mix(SinOsc.ar(freq*[1, 2], mul: amp*hc_amp));
        sig = 0.5 * (sig.pow((0.8-vio) + 0.2) / (1 + (10*vio.squared))) + (0.5 * sig);
        sig = LPF.ar(sig, 100 + (1000*dark));
        sig = sig + Mix(SinOsc.ar(freq*[8, 32, 64], mul: amp*hc_amp*dark.linexp(0.5, 1, 0.0001, 0.02)));
    },
    planet_spec: ~planet_specifications[\f],
    observer_spec: ~planet_specifications[\e]
);

~makePlanet.value(\g_from_e,
    { | amp, vio, dark, freq_multiplier, freq, wait, hc_amp=0.12 |
        var sig, steady_rhumble_sig, thunder_sig, howl_sig, grain_sig;
        var trig, graindur, cirrusfreq, storm, envbuf, winenv, sr_amp, th_amp, hw_amp, gr_amp;

        /*vio = MouseX.kr();
        dark = MouseY.kr();*/
        storm = dark;
        freq = Lag.kr(freq, wait)*freq_multiplier;

        howl_sig = LPF.ar(
            in: Mix(BPF.ar(
                in: WhiteNoise.ar(),
                freq: freq*LFNoise2.kr(8+(10*vio)).range(0.3, 3)*[4, 8, 10, 13, 16, 18],
                rq: dark.range(0.1, 3)
            )),
            freq: 600,
            mul: 0.3
        );

        steady_rhumble_sig = LPF.ar(
            in: Mix(BPF.ar(
                in: WhiteNoise.ar(),
                freq: freq*[1, 2, 3, 4, 5, 6, 7],
                mul: [1, 0.9, 0.8, 0.7, 0.6],
                rq: 0.05 + (0.1*dark.exp)
            )),
            freq: 600,
            mul: 3
        );

        thunder_sig = RLPF.ar(
            in: LFTri.ar(freq: WhiteNoise.kr().exprange(10, 300)),
            freq: LFNoise0.kr(freq: vio.linlin(0, 1, 5, 100)).exprange(10, 1000),
            mul: 0.7
        );

        // grain
        winenv = Env([0, 1, 0.7, 0], [0.1, 0.7, 0.2], \cub, 1);
        envbuf = Buffer.sendCollection(s, winenv.discretize, 1);

        cirrusfreq = Lag.kr(TExpRand.kr(0.3, 2, Dust.kr(0.1)), 30);
        graindur = LFNoise1.kr(LFNoise0.kr(0.2).range(0.1, 1)).range(0.1, 1);
        trig = ((1-storm)*Impulse.kr(cirrusfreq)) + (storm*Dust.kr(cirrusfreq));

        grain_sig = GrainSin.ar(
            numChannels: 1,
            trigger: trig,
            dur: (3/cirrusfreq) + (storm*graindur),
            freq: freq*TChoose.kr(trig, [1, 1, 1, 1, 1, 1, 1.5, 1.5, 2, 2, 3])*LFNoise0.kr(10).range(0.95, 1.05),
            envbufnum: envbuf,
        );

        grain_sig = (grain_sig + grain_sig.squared + grain_sig.cubed)*0.3;
        grain_sig = grain_sig.distort + (0.5*LPF.ar(FreeVerb.ar(grain_sig + (storm*Fold.ar(grain_sig, vio-1, 1-vio)), 0.5, 10), freq*10));

        gr_amp = 0.5;
        sr_amp = storm.linlin(0.3, 0.7, 0, 0.5) * storm.linlin(0.6, 1, 1, 0);
        hw_amp = storm.linlin(0.7, 1, 0, 0.5);
        th_amp = vio.linlin(0.5, 1, 0, 0.5);

        sig = ((th_amp*thunder_sig) + (sr_amp*steady_rhumble_sig) + (hw_amp*howl_sig) + (gr_amp*grain_sig))*amp*hc_amp;
    },
    planet_spec: ~planet_specifications[\g],
    observer_spec: ~planet_specifications[\e]
);

~makePlanet.value(\h_from_e,
    { | amp, vio, dark, freq_multiplier, freq, wait, hc_amp=0.1 |
        var sig, steady_rhumble_sig, thunder_sig, howl_sig, grain_sig;
        var trig, graindur, cirrusfreq, storm, envbuf, winenv, sr_amp, th_amp, hw_amp, gr_amp;

        /*vio = MouseX.kr();
        dark = MouseY.kr();*/
        storm = dark;
        freq = Lag.kr(freq, wait)*freq_multiplier;

        howl_sig = LPF.ar(
            in: Mix(BPF.ar(
                in: WhiteNoise.ar(),
                freq: freq*LFNoise2.kr(8+(10*vio)).range(0.3, 3)*[4, 8, 10, 13, 16, 18],
                rq: dark.range(0.1, 3)
            )),
            freq: 600,
            mul: 0.3
        );

        steady_rhumble_sig = LPF.ar(
            in: Mix(BPF.ar(
                in: WhiteNoise.ar(),
                freq: freq*[1, 2, 3, 4, 5, 6, 7],
                mul: [1, 0.9, 0.8, 0.7, 0.6],
                rq: 0.05 + (0.1*dark.exp)
            )),
            freq: 600,
            mul: 3
        );

        thunder_sig = RLPF.ar(
            in: LFTri.ar(freq: WhiteNoise.kr().exprange(10, 300)),
            freq: LFNoise0.kr(freq: vio.linlin(0, 1, 5, 100)).exprange(10, 1000),
            mul: 0.7
        );

        // grain
        winenv = Env([0, 1, 0.7, 0], [0.1, 0.7, 0.2], \cub, 1);
        envbuf = Buffer.sendCollection(s, winenv.discretize, 1);

        cirrusfreq = Lag.kr(TExpRand.kr(0.3, 2, Dust.kr(0.1)), 30);
        graindur = LFNoise1.kr(LFNoise0.kr(0.2).range(0.1, 1)).range(0.1, 1);
        trig = ((1-storm)*Impulse.kr(cirrusfreq)) + (storm*Dust.kr(cirrusfreq));

        grain_sig = GrainSin.ar(
            numChannels: 1,
            trigger: trig,
            dur: (3/cirrusfreq) + (storm*graindur),
            freq: freq*TChoose.kr(trig, [1, 1, 1, 1, 1, 1, 1.5, 1.5, 2, 2, 3])*LFNoise0.kr(10).range(0.95, 1.05),
            envbufnum: envbuf,
        );

        grain_sig = (grain_sig + grain_sig.squared + grain_sig.cubed)*0.3;
        grain_sig = grain_sig.distort + (0.5*LPF.ar(FreeVerb.ar(grain_sig + (storm*Fold.ar(grain_sig, vio-1, 1-vio)), 0.5, 10), freq*10));

        gr_amp = 0.5;
        sr_amp = storm.linlin(0.3, 0.7, 0, 0.5) * storm.linlin(0.6, 1, 1, 0);
        hw_amp = storm.linlin(0.7, 1, 0, 0.5);
        th_amp = vio.linlin(0.5, 1, 0, 0.5);

        sig = ((th_amp*thunder_sig) + (sr_amp*steady_rhumble_sig) + (hw_amp*howl_sig) + (gr_amp*grain_sig))*amp*hc_amp;
    },
    planet_spec: ~planet_specifications[\h],
    observer_spec: ~planet_specifications[\e]
);


MIDIClient.init;
MIDIIn.connectAll;
MIDIdef.freeAll;

// Connect control busses to MIDI inputs:
/* MIDI channel number is planet number. Each planet has four control busses, addressed by MIDI CC numbers 102, 103, 104, 105 */
MIDIdef.cc("midicclin", {
    arg val, num, chan, src;
	// [val, num, chan, src].postln;
    ~planetsControlBusses[chan].setAt(num-102, val/127);
},
// ccNum: [102, 103, 104, 105],
ccNum: [103, 104],
);

MIDIdef.cc("midirate", {
    arg val, num, chan, src;
    var rate;
	// [val, num, chan, src].postln;

    rate = LinLin.kr(val, 0, 127, 1, 10);
    ~rateControlBus.set(rate);
},
ccNum: [105],
);


// Instantiate and play synths:
~synths = Array.newClear(~numPlanets);
~synths[0] = Synth.new(\b_from_e, [\freq, 50], ~planetsGroup);
~synths[1] = Synth.new(\c_from_e, [\freq, 80], ~planetsGroup);
~synths[2] = Synth.new(\i_from_e, [\freq, 100], ~planetsGroup);
~synths[3] = Synth.new(\d_from_e, [\freq, 200], ~planetsGroup);
~synths[4] = Synth.new(\f_from_e, [\freq, 50], ~planetsGroup);
~synths[5] = Synth.new(\g_from_e, [\freq, 40], ~planetsGroup);
~synths[6] = Synth.new(\h_from_e, [\freq, 60], ~planetsGroup);

// Couple control busses to synth args:
~synths.do {
    arg item, i;
	if ( item.isNil.not, {
		item.map(
            // \amp, ~planetsControlBusses[i].subBus(0),
			\vio, ~planetsControlBusses[i].subBus(1),
			\dark, ~planetsControlBusses[i].subBus(2),
            \playback_rate, ~rateControlBus
		);
	});
};

// s.meter;
) /* END SETUP MAIN */



r = Recorder(s);
r.record(bus: ~hoaBus.index, numChannels: ~hoaNumChannels.asInt, duration: 20*60);
r.record(numChannels: 2);
r.stopRecording;

~planetsGroup.set(\playback_rate, 20);

~rateControlBus.scope;

~hoaBus.index
~hoaBus.scope;
~recBus.scope;
