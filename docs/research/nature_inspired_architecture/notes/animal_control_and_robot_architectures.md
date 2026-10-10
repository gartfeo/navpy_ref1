# Animal Motor Control and Bio-inspired Robot Control Architectures (for judging Mission -> Skills -> Moves -> Reflexes)

Scope: (a) how vertebrate (and some insect) nervous systems layer motor control, (b) robot architectures that copied that layering, their deployments, results, failures, and (c) lessons for a small team building fixed-wing UAV autonomy on ArduPilot with a companion computer. Research date: 2026-10-09. Sources: abstracts plus full text where retrieved (Gat 1998 and the DS1 Remote Agent validation paper were read in full). Attribution caveats are flagged inline.

---

## Q1. Biology: how do spinal reflexes, CPGs, motor primitives/synergies, brainstem, cerebellum, basal ganglia and cortex divide the work, and how do higher levels modulate rather than micromanage?

### Takeaway
Vertebrate motor control is a stack in which each level owns a complete, self-stabilising competence, and the level above mostly selects, starts/stops, and parameterises it (speed, gain, goal). It does not stream muscle-level commands. Spinal CPGs and reflexes handle timing and perturbations. Brainstem command centres set the drive level. Basal ganglia pick which program runs. The cerebellum supplies predictions (forward models) that hide feedback delays. Cortex specifies and tunes task-specific feedback.

### Cited Findings
**Spinal level: CPGs, reflexes, primitives**
- In Grillner & El Manira's review of vertebrate locomotion, the core propulsion system is spinal CPG networks that time different muscles and compensate for perturbations. Brainstem command systems set how active the CPGs are, and therefore locomotor speed. — [Grillner & El Manira, Physiol. Rev. (abstract via reference listing)](https://insight.jci.org/references/scholar/3959/B30)
- Grillner proposed separate CPGs for each limb segment, interconnected to coordinate the whole movement. Spinal networks produce alternating flexor/extensor bursts from intrinsic cell and network properties. — [Frontiers Neurosci. 2017 review](https://www.frontiersin.org/journals/neuroscience/articles/10.3389/fnins.2017.00581/pdf); [Grillner lab](https://www.neuro.ki.se/ki-imitation/grillner.html)
- Frog spinal microstimulation produced a small set of convergent force fields ("primitives"). Co-stimulating two sites gave roughly the vector sum of the separate fields: equivalent in 83% of cases in Mussa-Ivaldi et al. 1994, though another review reports >87%. Scaling and summing a few field types can generate a large range of force-field structures. — [Bizzi & Cheung 2013, Front. Comput. Neurosci.](https://www.frontiersin.org/journals/computational-neuroscience/articles/10.3389/fncom.2013.00051/full); [PMC3638124](https://pmc.ncbi.nlm.nih.gov/articles/PMC3638124)
- Muscle activity in intact and spinalised animals can be reconstructed as linear combinations of a few fixed activation patterns (synergies), and the spinal cord "constructs" movement from these modules. — [Tresch, Saltiel & Bizzi 1999, Nat. Neurosci.](https://www.nature.com/articles/nn0299_162); [Bizzi & Cheung 2013](https://www.frontiersin.org/journals/computational-neuroscience/articles/10.3389/fncom.2013.00051/full)
- A 2005 MIT thesis argues that frog spinal circuitry fits a control scheme built on a reduced-order model of the musculoskeletal system, with primitives corresponding to synergies. — [MIT DSpace thesis](https://dspace.mit.edu/handle/1721.1/33919)

**Brainstem: drive and start/stop**
- In lamprey (and largely other vertebrates), reticulospinal neurons carry most of the descending drive that activates spinal locomotor CPGs. The basal ganglia project to the mesencephalic locomotor region (MLR), which projects to reticulospinal cells. Basal ganglia and hypothalamus also project directly to MLR and reticulospinal neurons. A second region (DLR) is poorly understood. — [Frontiers Neural Circuits 2023 mini-review](https://www.frontiersin.org/journals/neural-circuits/articles/10.3389/fncir.2023.910207/full)
- Much of this circuit evidence comes from basal vertebrates (lamprey), and mammalian details may differ. — [same](https://www.frontiersin.org/journals/neural-circuits/articles/10.3389/fncir.2023.910207/full)

**Basal ganglia: action selection**
- Redgrave, Prescott & Gurney (1999) frame selection as a problem that arises "whenever two or more competing systems seek simultaneous access to a restricted resource". They argue that a central switching mechanism has significant advantages over distributed alternatives and that the basal ganglia evolved as such a centralised selection device for motor and cognitive resources. — [Redgrave, Prescott & Gurney 1999, Neuroscience 89(4)](https://org.osu.edu/cognitive-science-club/files/2020/01/RedgravePrescottGurney99.pdf); [White Rose record](https://eprints.whiterose.ac.uk/107033)
- Computational models (Gurney et al. 2001) show the BG circuit can resolve competition so that its output expresses the most appropriate action(s) and suppresses the others. The same group later acknowledged that the BG are not the complete vertebrate action-selection system. — [PMC2440776](https://pmc.ncbi.nlm.nih.gov/articles/PMC2440776)
- In Grillner's account, the forebrain, especially the basal ganglia, decides which motor programs are recruited at a given time and can both initiate and stop locomotion. — [Grillner & El Manira (abstract)](https://insight.jci.org/references/scholar/3959/B30)

**Cerebellum: internal models**
- Wolpert, Miall & Kawato (1998): forward models predict the consequences of actions "and can be used to overcome time delays associated with feedback control". Inverse models supply the command for a desired state change, so they are "well suited to act as controllers". They review evidence for inverse models in cerebellar circuitry (ocular following) and for cerebellar forward-model predictions, and propose multiple paired forward/inverse models. — [Wolpert, Miall & Kawato 1998, TICS 2(9):338-347](https://wolpertlab.neuroscience.columbia.edu/sites/default/files/content/papers/WolMiaKaw98.pdf)

**Cortex: specifying and tuning feedback rather than issuing trajectories**
- Optimal feedback control (Todorov & Jordan 2002; Scott 2004): the nervous system chooses time-varying feedback gains suited to the task, minimises a cost (accuracy, effort), and corrects only task-relevant variability. Each task needs its own feedback law, which must be selected. — [Scott 2002 commentary](https://homes.cs.washington.edu/~todorov/papers/ScottNatNeurosci02_news.pdf); [Scott 2004, Nat. Rev. Neurosci. 5:532 (DOI)](https://doi.org/10.1038/nrn1427); [Diedrichsen, Shadmehr & Ivry 2010](https://ivrylab.berkeley.edu/files/organized_pubs_pdfs/2010_diedrichsen_shadmehr_ivry.pdf)
- Cisek's affordance-competition hypothesis: action selection (which) and specification (how) run in parallel. Multiple concrete motor plans are prepared at once in fronto-parietal circuits and compete by mutual inhibition. Movement planning is "the basis for" the decision rather than its output, so no dedicated decision module is needed. One caveat: most supporting experiments used cued rather than self-chosen actions. — [Cisek 2007, Phil. Trans. R. Soc. B (PMC2440773)](https://pmc.ncbi.nlm.nih.gov/articles/PMC2440773); [U. Alberta thesis caveat](https://era.library.ualberta.ca/items/88f88122-c387-4d66-97c3-e3457b040eed)

### Inferences
- The recurring biological pattern is "select + parameterise + let the lower loop close itself". Brainstem drive sets CPG speed, the basal ganglia choose the program, and cortex sets feedback gains and goals. A higher level that streams low-level commands is the exception (for example, fine dexterous control), not the rule. This maps directly onto Skills choosing and parameterising Moves (goto, orbit, hold, climb) that the autopilot closes itself.
- Primitives combine roughly additively (force-field summation), so a small vocabulary of well-behaved Moves can generate a large repertoire. That argues for keeping the Moves set small and making each Move internally stable.
- Prediction (cerebellum) is a separate concern from selection (basal ganglia) and from execution (spinal). In a UAV stack, prediction and estimation are services consulted by several layers, not a layer of their own.

### Gaps
- I did not retrieve full texts of Grillner & El Manira (Physiol. Rev.) or Scott 2004. Claims rest on abstracts and citing papers.
- Not retrieved: the classic tonic-inhibition/disinhibition mechanism of BG output (GPi/SNr) or Ijspeert's CPG-driven salamander robot. Both are standard but uncited here.

---

## Q2. Biology: where do attention and a "world model" live; how do reflexes override intentions, and when do intentions suppress reflexes?

### Takeaway
There is no single world-model module. Prediction lives in the cerebellum, spatial orienting and attention priority run through the superior colliculus with cortex, and insects keep a dedicated heading estimator (central complex ring attractor). Reflex/intention arbitration is not a fixed priority list: descending commands continuously gate reflex gain by phase, task and goal. Some reflexes even reverse sign depending on context.

### Cited Findings
**Attention and orienting**
- The superior colliculus (SC) "implements the motor consequences of attention" and plays "a crucial role in the process of target selection that precedes movement". SC activity tracks covert attention shifts without eye movement and is necessary for normal spatial attention. Its contribution operates through mechanisms independent of the known attention signatures in visual cortex. — [Krauzlis, Lovejoy & Zénon 2013, Annu. Rev. Neurosci. 36:165-182 (DOI)](https://doi.org/10.1146/annurev-neuro-062012-170249)
- A review concludes that priority/saliency-map-like signals exist in several areas, but the SC mechanistically implements the saliency-map model and is "the final gatekeeper" between priority maps and overt behaviour. — [PMC5206280](https://pmc.ncbi.nlm.nih.gov/articles/PMC5206280)
- Krauzlis describes the SC as a "neural pointer" to behaviourally relevant objects. — [Salk 2008 release](https://www.salk.edu/news-release/looking-versus-seeing/)
- In rat SC slices, an asymmetric inhibitory circuit hard-wires higher priority for more peripheral targets. — [Bayguinov et al. 2015 (WUSTL record)](https://profiles.wustl.edu/en/publications/a-hard-wired-priority-map-in-the-superior-colliculus-shaped-by-as/)

**World model / state estimation (insect example relevant to flying vehicles)**
- In Drosophila, a localised activity bump in the ellipsoid body (central complex) moves as the fly turns, acting as a heading estimate. It persists without external cues, locks onto one of two competing cues, and drifts in darkness. Recurrent coupling with angular-velocity neurons keeps it updated, much like the mammalian head-direction system. — [eLife 2020 (Pisokas, Heinze, Webb)](https://elifesciences.org/articles/53985); [PMC5328262](https://pmc.ncbi.nlm.nih.gov/articles/PMC5328262); [PMC7419142](https://pmc.ncbi.nlm.nih.gov/articles/PMC7419142)
- Modelling of fly and locust central complexes shows ring-attractor topology with eight-fold symmetry. The fly circuit responds faster to heading changes, and the locust circuit is more noise-tolerant. — [Pisokas, Heinze & Webb 2020, eLife](https://elifesciences.org/articles/53985)
- The landmark-to-compass offset differs between flies and can shift, which suggests landmarks are bound to the heading ring by plastic rather than fixed connections. — [PMC7419142](https://pmc.ncbi.nlm.nih.gov/articles/PMC7419142)

**Intentions suppress or modulate reflexes**
- Spinal reflex pathways form basic circuits that descending commands modulate during voluntary movement. Their feedback must be either combined with or suppressed by the motor command, and the earliest suppression point is presynaptic inhibition. — [Fetz lab review, Curr. Opin.](https://depts.washington.edu/fetzweb/assets/curropinfpp.pdf)
- In awake monkeys, cutaneous input to spinal interneurons is presynaptically inhibited during active wrist movement, and descending commands produce this inhibition. — [Seki, Perlmutter & Fetz 2003, Nat. Neurosci.](https://www.nature.com/articles/nn1154)
- Human leg H-reflex pathways are down-regulated by movement-elicited afferent discharge. Presynaptic inhibition persists after movement stops, and cutaneous reflex gains are modulated during active movement. — [Brooke et al. 1997 review](https://sites.ualberta.ca/~dcollins/articles/Brooke1997b.pdf)
- Cutaneous reflex excitability during locomotion changes with phase, nerve and task. The spinal pattern generator may implement this modulation, though human evidence is indirect. — [JPFSM 2012 review](https://doaj.org/article/dbbf48cdee614731a35ea35b91b93c32)
- Reflex timing patterns were largely conserved across level walking, incline walking and stair climbing, but the tibialis anterior reflex reversed during stair climbing, a task-specific sign flip. — [Lamont & Zehr 2006, Exp. Brain Res.](https://link.springer.com/doi/10.1007/s00221-006-0586-4)
- The long-latency stretch response is goal-dependent at the shoulder, elbow and wrist (title-level evidence only). — [J. Neurophysiol. 2015 record](https://www.peeref.com/works/877237)

**Reflexes override intentions**
- The spinal/brainstem layers keep compensating for perturbations whatever the higher command. In the review's account, the CPG layer includes "sensory compensation for perturbations" as part of its core function. — [Grillner & El Manira (abstract)](https://insight.jci.org/references/scholar/3959/B30)

### Inferences
- Biology has both directions of override, but they work differently. Reflexes protect continuously at low latency. Intentions do not switch reflexes off wholesale; they turn reflex gain up or down by context and phase, and may flip a reflex's sign for a specific task. For a UAV, this argues for (a) hard reflexes (geofence, failsafes, termination) that no upper layer can disable, and (b) soft reflexes whose thresholds or behaviour the active Skill can modulate within pre-declared, bounded, logged ranges. Example: a final-approach Skill could widen a "too close to object" margin rule only for the declared POI.
- The SC "final gatekeeper" role suggests attention and target selection belong close to the action selector, not in a remote planner. That fits a design where Skills own the choice of what the camera/gimbal attends to.
- The insect heading estimator shows a dedicated, always-running estimator that higher layers read but do not own. For NavPy, that matches keeping estimation separate from command (see the project's Pure Vision Approach constraint on which estimates the command path may consume).

### Gaps
- I did not retrieve a source on Forssberg's classic phase-dependent reflex reversal (stumbling-corrective reaction), on hippocampal place/grid cells as a mammalian "cognitive map", or on startle/escape circuits (e.g., Mauthner cell) as hard overrides. These are standard results but are uncited here.
- No single review was found that directly answers "when does intention suppress a reflex vs. when does the reflex win". The answer above is synthesised from separate modulation studies.

---

## Q3. Robotics: subsumption and behavior-based robotics (Brooks, Arkin/AuRA/motor schemas): core idea, deployments, what worked, what failed

### Takeaway
Subsumption showed that fast, layered, stateless reactive competences make robust low-level robots. It reached mass products through iRobot (Roomba, PackBot) and is a core idea in modern behavior trees. As a whole-system architecture it hit a "capability ceiling": upper layers interfering with lower layers' internals destroyed modularity, and pure wire-suppression arbitration did not scale. Arkin's motor schemas replaced priority suppression with vector summation and paired it with a deliberative layer (AuRA), at the cost of potential-field pathologies (local minima, oscillation).

### Cited Findings
**Brooks' subsumption (1986)**
- Brooks, "A Robust Layered Control System for a Mobile Robot", IEEE J. Robotics & Automation 2(1):14-23, 1986 (MIT AI Memo 864, 1985). Layers add levels of competence. Modules are asynchronous, connected by low-bandwidth messages, and computationally modest. Higher layers take over by suppressing or inhibiting lower-layer signals, and lower layers keep running. — [IEEE Xplore](https://ieeexplore.ieee.org/document/1087032); [PDF](https://courses.cs.washington.edu/courses/cse481c/09wi/papers/Brooks-RA86-RobustLayeredControlSystem.pdf)
- Gat notes that subsumption layers are networks of small finite-state machines joined by wires, and that its only composition mechanism is "the ability to override the contents of one wire with a value from another wire". Subsumption also "provides no architectural mechanism to enforce (or even support)" its layering methodology. — [Gat 1998](https://flownet.com/gat/papers/tla.pdf) (alt. copy [BIU](https://u.cs.biu.ac.il/~galk/teach/current/intsys/readings/on-three-layer-arch-tla-1998.pdf))
- Early success: "While SPA-based robots were pondering their plans, Subsumption-based robots were zipping around the lab". — [Gat 1998](https://flownet.com/gat/papers/tla.pdf)
- Capability ceiling: Herbert (soda-can retrieval) was the pinnacle, but "there is no record of it ever having performed a complete can-retrieval task flawlessly", and no subsumption robot had matched it since (as of 1998). — [Gat 1998](https://flownet.com/gat/papers/tla.pdf)
- Hartley & Pipitone (ICRA 1991), quoted by Gat: "The most important problem we found with the Subsumption architecture is that it is not sufficiently modular… Because upper layers interfere with the internal functions of lower-level behaviors they cannot be designed independently and become increasingly complex… even small changes to low-level behaviors or to the vehicle itself cannot be made without redesigning the whole controller." Also: "communicate through the world" was rarely useful because "very similar states of the world could mean different things depending on the context". Also: "Determining that one behavior is more high-level than another is sometimes completely artificial… Sometimes the low level should override higher levels." — [Gat 1998](https://flownet.com/gat/papers/tla.pdf)
- Contrast: Tooth and Rocky III (JPL, ~1989-90) were layered like subsumption, but higher layers gave input or advice to lower ones instead of suppressing their outputs. They were "extremely reliable, running many dozens of trials without failing" on 8-bit microcontrollers with about 2000 bytes of memory each. Their drawback was that they were not taskable without rewriting code. — [Gat 1998](https://flownet.com/gat/papers/tla.pdf)
- The behavior-tree literature lists scalability and maintainability as subsumption's weaknesses: complex action selection through distributed inhibition and suppression is hard to design, and the effects of adding or removing controllers are hard to estimate. — [Colledanchise & Ögren, Behavior Trees in Robotics and AI (arXiv 1709.00084)](https://arxiv.org/pdf/1709.00084); see also [arXiv 1611.05379](https://arxiv.org/pdf/1611.05379)
- A 1994 UPenn report characterises subsumption as assuming limited knowledge of the environment, no explicit representation, limited reasoning and no centralised control. — [UPenn IRCS report](https://repository.upenn.edu/ircs_reports/178)

**Deployment: iRobot**
- Brooks co-founded iRobot in 1990. Over 10 million Roombas had been sold, and the IEEE RAS credits subsumption with enabling the human-robot interaction that the Roomba exemplifies (a tribute claim, not a technical analysis). — [IEEE RAS 2023 Founders Medal note](https://www.ieee-ras.org/about-ras/latest-news/congratulations-to-rodney-brooks-recipient-of-the-2023-ieee-founders-medal)
- Joseph Jones (iRobot) describes Roomba control as a behavior-based program on a low-cost microprocessor. A minimal version has two behaviors, Cruise and Escape (bumper-triggered). Around 2000, position sensing would have added more than $1,000 to the cost, so the robot had to clean without knowing where it was. This is an illustrative design, not a published firmware description. — [Jones, "Robots at the tipping point: the road to iRobot Roomba"](https://researchgate.net/profile/Joseph_Jones13/publication/3344755_Robots_at_the_tipping_point_the_road_to_iRobot_Roomba/links/5728aa7308ae2efbfdb7dce8/Robots-at-the-tipping-point-the-road-to-iRobot-Roomba.pdf)
- PackBot came out of DARPA work (Urbie, 1997). It was first deployed militarily in Afghanistan in July 2002 and later used at the WTC and Fukushima. Fleet figures vary by source (>2,000 in Iraq/Afghanistan; >5,000 delivered). Note that PackBot is mostly teleoperated, so it is weak evidence for autonomy architecture. — [Wikipedia: PackBot](https://en.wikipedia.org/wiki/PackBot)

**Arkin: motor schemas and AuRA**
- Arkin, "Motor Schema-Based Mobile Robot Navigation", IJRR 8(4):92-112, 1989. Schemas run as concurrent processes, each with embedded perceptual schemas supplying only what that behavior needs. Each outputs a velocity vector (potential-field analog), and the vectors are summed to give the commanded speed and heading. — [Arkin 1989 PDF](https://www.ini.rub.de/upload/file/1527093387_3a981e55d1876c5e1a88/Arkin89.pdf)
- AuRA (Georgia Tech, mid-1980s) is a hybrid deliberative/reactive architecture: the deliberative layer selects which behaviors are active for each context. — [Wikipedia: Autonomous robot architecture](https://en.wikipedia.org/wiki/Autonomous_robot_architecture)
- Potential-field methods exhibit local minima and cyclic behaviour, which is one reason AuRA adds deliberative planning (from a secondary paper; attribution to a specific page in the result set is uncertain). — [IFAC 2002 paper](https://skoge.folk.ntnu.no/prost/proceedings/ifac2002/data/content/00351/351.pdf); [Goodrich potential-fields notes](https://phoenix.goucher.edu/~jillz/cs325_robotics/goodrich_potential_fields.pdf)
- Balch & Arkin formation-keeping behaviors were implemented as motor schemas in AuRA and as steering/speed behaviors in the DARPA UGV Demo II architecture (slide-deck source). — [Georgia Tech lecture slides](https://sites.cc.gatech.edu/home/dellaert/07F-Robotics/Schedule_files/03-BehaviorBasedControl.ppt.pdf)

### Inferences
- What scaled from subsumption was the bottom layer: always-on, low-state, fast safety behaviors (Escape, collision stop). What did not scale was using suppression wires as the general composition and arbitration mechanism.
- Hartley & Pipitone's "sometimes the low level should override higher levels" is exactly the Reflexes-on-top requirement in the proposed UAV stack. Subsumption's fixed "higher layer wins" rule gets this backwards for safety.
- Vector-summing arbitration (motor schemas) is attractive for blending (e.g., obstacle repulsion plus goal attraction), but it can produce commands that no behavior intended and can trap the vehicle. For a fixed-wing aircraft with a minimum airspeed and turn radius this risk is higher. My opinion: prefer selection (one Move active) over blending at the Skills-to-Moves boundary, and blend only inside a Move whose closed-loop behaviour has been analysed.

### Gaps
- I found no authoritative source documenting actual Roomba firmware architecture or PackBot autonomy architecture.
- I could not retrieve Arkin's *Behavior-Based Robotics* (MIT Press 1998) text, documented AuRA/MissionLab field deployments beyond UGV Demo II, or quantified failure rates.
- The "MARS" results in searches referred to a 2025 Army mine-breaching demo (DARPA RACER vehicle), unrelated to AuRA.

---

## Q4. Robotics: three-layer and hybrid deliberative-reactive architectures (Gat/ATLANTIS, 3T, Remote Agent, CLARAty, ASE/EO-1, NIST 4D/RCS, WITAS/HDRC3, behavior trees): what was deployed, what worked, what failed

### Takeaway
The three-layer architecture (controller / sequencer / deliberator) became the de facto standard because it organises code by how it uses internal state. It has long, real deployments: 3T ran NASA JSC life support 24/7 for 16 months, ASE flew EO-1 operations for 12+ years, and DS1 Remote Agent met 100% of its objectives. The recurring failure points are in the middle (executive/sequencer): concurrency bugs (the DS1 deadlock) and executive/planner state drifting out of sync with the vehicle. Successful flight systems kept an independent, simple fault-protection layer below all the AI.

### Cited Findings
**Gat 1998, "On Three-Layer Architectures" (in Kortenkamp, Bonasso & Murphy eds., *AI and Mobile Robots*, AAAI Press)** — all from [Gat 1998 full text](https://flownet.com/gat/papers/tla.pdf)
- At least three groups converged independently (Connell SSS, Bonasso 3T, Gat ATLANTIS) on a reactive feedback controller, a slow deliberative planner, and a sequencer connecting them. The earliest description is in Firby's 1989 RAPs thesis.
- Organising principle is internal state: "Stateless sensor-based algorithms inhabit the control component. Algorithms that contain memory about the past inhabit the sequencer. Algorithms that make predictions about the future inhabit the deliberator." SPA "gets into trouble when its internal state loses sync with the reality that it is intended to represent".
- Controller constraints:
  - Each iteration runs in constant-bounded time and space.
  - Algorithms "should fail cognizantly… Rather than attempt to design algorithms that never fail… design algorithms that never fail to detect a failure".
  - Internal state is avoided, or "ephemeral" (expires after bounded time), except for filters.
  - No state-induced discontinuities: "It is the responsibility of the sequencer to manage transitions between regimes of continuous operation."
- Sequencer: selects which primitive behavior is active and supplies its parameters. It responds conditionally to the actual outcome (conditional sequencing: RAPs, PRS, ESL). It "should not perform any search or temporal projection".
- Deliberator: "several Behavior transitions can occur between the time a deliberative algorithm is invoked and the time it produces a result". It either produces plans (3T) or answers sequencer queries (ATLANTIS).
- "Running researcher syndrome": SPA plan steps executed in the wrong context after an unexpected outcome.
- Case study (Alfred, 1993 AAAI contest): an always-running collision/hard-obstacle safety routine lived in the controller. Any obstacle slowed the robot immediately, but only "a succession of clear readings" sped it back up (asymmetric hysteresis). All primitives were under 200 lines. Contest-specific code was "written in three days by one person". The deliberator was "trivial and uninteresting, which is precisely what makes the three-layer architecture non-trivial".
- Pragmatic blurring: collision response was put in the controller rather than the sequencer because latency mattered mechanically. Gat: "the lines between the components… can be blurred to accommodate reality."
- Self-stated limits: guidelines are "derived from empirical observations", not theory. The architecture "largely ignores… sensor processing, learning, and world modeling". Attempts at languages that enforce controller constraints (ALFA) "have been largely unsuccessful", and Gat prefers C or Lisp "and a little self-discipline".

**3T at NASA JSC (Bonasso, Kortenkamp et al.)**
- 3T tiers: a hierarchical task-net planner (deliberative), a reactive sequencer (RAPs) that builds procedures from situational context, and a skill manager. 3T controlled the JSC water Post Processing System. For the integrated water recovery system it handled 200+ sensors and actuators across four subsystems and "operated autonomously, 24/7 for sixteen months". Earlier it ran a biological water processor 24 hours a day for 12 months. (Quotes come from TRACLabs/AAAI papers in the result set; exact page attribution between them is uncertain.) — [Bonasso, Kortenkamp & Thronesbery, AI Magazine 2003 "Three years in the trenches"](https://traclabs.com/wp-content/uploads/2024/05/ai_mag2003.pdf); [AAAI WS-04-05](https://www.aaai.org/Papers/Workshops/2004/WS-04-05/WS04-05-005.pdf)
- The authors report events "for which we were ill prepared" in long-duration life support control (full text not read). — [AI Magazine 2003](https://traclabs.com/wp-content/uploads/2024/05/ai_mag2003.pdf)
- Human-in-the-loop needs emerged: operator liaison agents (Ariel) were built for situation summarisation, event notification and support for manual commanding of the autonomous system. — [TRACLabs DCI paper](https://traclabs.com/wp-content/uploads/2024/05/ieee_dci.pdf)

**NASA Deep Space 1 Remote Agent (RAX, May 1999)** — from [Nayak et al., "Validating the DS1 Remote Agent Experiment", iSAIRAS 1999](https://ai.jpl.nasa.gov/public/documents/papers/rax-results-isairas99.pdf) unless noted
- Architecture: Mission Manager plus Planner/Scheduler (deliberative), a multi-threaded Smart Executive (sequencer), and Livingstone Mode Identification and Reconfiguration (model-based diagnosis), above conventional real-time flight software. — same; [Muscettola et al. 1998, AIJ](https://groups.csail.mit.edu/mers/old-site/abstracts/aij98-abstract.html)
- Testing: a pyramid of testbeds from fast low-fidelity simulators to flight-spare hardware. Automated overnight runs found and fixed "over 800 bugs in six months".
- In-flight failure: on 18 May 1999 RAX did not command termination of ion-engine thrusting. The cause was "a missing critical section in the plan execution code", which created "a race condition between two EXEC threads". If the wrong thread won, each waited on the other (deadlock). This "had not occurred even once in thousands of previous races on the various ground platforms". Roughly 70% of objectives were met by then. Spacecraft health was unaffected, and the experiment was stopped by ground.
- Recovery: a patch was built, but the DS1 project declined to uplink it, "citing insufficient testing". Instead a new 6-hour scenario was designed and tested in about 10 hours, and RAX ultimately achieved "100%" of its validation objectives.
- Second anomaly: supporting software failed to confirm an ion-engine state transition, so RA "correctly" stopped the startup sequence. The resulting RA state was inconsistent with spacecraft state, but the discrepancy happened to be benign.
- Pre-flight model checking with SPIN found five concurrency errors early in design that testing would likely have missed. This required costly manual translation from Lisp to PROMELA and motivated Java PathFinder. — [Havelund et al., "Formal Analysis of the Remote Agent Before and After Flight", NTRS](https://ntrs.nasa.gov/api/citations/20000055731/downloads/20000055731.pdf)
- The race was debugged on the live spacecraft through a Lisp REPL (secondary account, quoting Garret's "Lisping at JPL"). — [susam.net](https://susam.net/very-remote-debugging.html)

**CLARAty (JPL/Ames/CMU, from 1999)**
- CLARAty was proposed as an "evolutionary modification of the conventional three-level robotics architecture". It has two tiers: a Decision Layer that tightly couples planner and executive, and a Functional Layer (system operation, resource prediction, state estimation, status reporting) that the Decision Layer can access "at all levels of system granularity". Target platforms were the Rocky 7/8 research rovers. — [Volpe et al. 2001](https://www-robotics.jpl.nasa.gov/media/documents/01_volpe_claraty_aerospace.pdf); [Estlin et al., iSAIRAS 2001](https://ai.jpl.nasa.gov/public/papers/isairas01-estlin.pdf)
- Rationale: in classic three-layer designs the planner was cut off from functional-layer information, so planners kept their own duplicate internal model of the robot. — [Nesnas et al. 2006, IJARS](https://journals.sagepub.com/doi/10.5772/5766)

**Autonomous Sciencecraft Experiment (ASE) on EO-1**
- Three layers:
  - CASPER planner, working on a timescale of tens of minutes.
  - SCL executive, on several-second timescales with event-driven low-level autonomy.
  - Flight software, handling low-level control and running "an independent layer of fault protection".
  - Onboard science analysis feeds new observation requests back to CASPER.
  - Budget: about 4 MIPS and 128 MB RAM. — [Sherwood et al., SpaceOps 2006](https://ai.jpl.nasa.gov/public/papers/sherwood-spaceops06-autonomy.pdf); [Chien et al., IWPSS 2006](https://www.stsci.edu/largefiles/iwpss/2006620252chien-s-iwpss2006.pdf)
- Operations: experimental from 2003/2004 and primary operations system from November 2004, a 2005 NASA Software of the Year co-winner. It guided EO-1 for "more than 12 years". The team self-reports a 100x increase in science return per downlinked data and more than $1M/year in operations savings. — [Sherwood et al. SmallSat 2007](https://ai.jpl.nasa.gov/public/documents/papers/sherwood-smallsat07-eo1.pdf); [Geology In 2017](https://www.geologyin.com/2017/03/how-ai-captured-volcanos-changing-lava.html); [Spaceflight101](https://spaceflight101.com/?p=35260)

**Mars rovers (status 2024-2026)**
- More than 90% of Perseverance's driving has relied on autonomous navigation (ENav), against about 6.2% for Curiosity. Earlier rovers drove under 25 m/h, and Perseverance's autonomous mode drives about 100 m/h. — [JPL news](https://www.jpl.nasa.gov/news/autonomous-systems-help-nasas-perseverance-do-more-science-on-mars/); [IEEE Spectrum](https://spectrum.ieee.org/perseverance-mars-rover-autonomous-driving)

**NIST 4D/RCS (Albus)**
- 4D/RCS is a multi-layered, multi-resolution hierarchy of nodes. Each node contains sensory processing, world modeling, value judgment and behavior generation. Lower echelons produce goal-seeking reactive behavior and higher echelons goal-defining deliberative behavior. It merged NIST RCS with the German VaMoRs 4-D dynamic-vision approach. It was developed with ARL and the German MoD for Army Demo III XUVs. — [NISTIR 5994](https://nvlpubs.nist.gov/nistpubs/Legacy/IR/nistir5994.pdf) ([DOI](https://doi.org/10.6028/NIST.IR.5994))
- Demo III XUVs planned and re-planned their own routes, speed and gaze. They refined soldiers' high-level commands with 30 m DTED and onboard sensors, and soldiers tested them on scouting scenarios. 2002-03 NIST/ARL engineering tests reported Demo III autonomous mobility met TRL 6 (NIST abstract pages; exact page attribution uncertain). Later work targeted tactical behaviors for ground and air vehicles. — [NIST tactical-behavior publication](https://www.nist.gov/publications/intelligent-control-and-tactical-behavior-development-long-term-nist-partnership-army-0); [NIST record](https://www.nist.gov/node/706091)

**UAV-specific hybrid: WITAS / HDRC3 (Linköping)**
- WITAS (1997-2005, 20+ researchers) used a three-layer hybrid deliberative/reactive architecture with layers as separate asynchronous processes and a CORBA backbone. Reported flights include autonomously tracking a moving vehicle for up to 20 minutes and self-planning a facade-photography mission. — [WITAS architecture page](https://www.ida.liu.se/ext/witas/info/arch/display.html); [WITAS demo paper 2006](https://www.ida.liu.se/divisions/aiics/publications/ICAPSSD-2006-WITAS-UAV-Ground.pdf); [project page](https://www.ida.liu.se/divisions/aiics/projects/witas.en.shtml)
- HDRC3 (Doherty et al., *Handbook of UAVs* 2014, pp. 849-952) uses Hierarchical Concurrent State Machines for low-level reactive control, Task Specification Trees for high-level tasks, integrated task and motion planners, stream-based data flow, and temporal-logic execution monitoring. It was used on a Yamaha RMAX helicopter and the LinkQuad quadrotor. — [HDRC3 chapter PDF](https://www.ida.liu.se/divisions/aiics/publications/Chapter-2014-HDRC3-Distributed-Hybrid.pdf); [ICSENG 2017](https://www.ida.liu.se/divisions/aiics/publications/ICSENG-2017-Bridging-Reactive-Control.pdf)

**Behavior trees (modern descendant of the sequencer layer)**
- A survey of 160+ BT papers in games and robotics found that BTs were adopted because finite-state machines scaled poorly and were hard to extend, adapt and reuse. — [Iovino et al. 2022, RAS 154 (arXiv 2005.05842)](https://arxiv.org/abs/2005.05842)
- Colledanchise & Ögren argue that BTs are modular and reactive, and include a formal analysis of how BTs generalise earlier ideas, including subsumption. — [arXiv 1709.00084](https://arxiv.org/pdf/1709.00084)

**Survey references**
- Kortenkamp & Simmons, "Robotic Systems Architectures and Programming", *Springer Handbook of Robotics* (2008; 2016 edition with Brugali, pp. 283-306, DOI 10.1007/978-3-319-32552-1_12). Only bibliographic data was retrieved. — [marineinfo record](https://marineinfo.org/doc/publication/294911); [Kortenkamp publications](https://traclabs.com/people/dr-dave-kortenkamp/)

### Inferences
- The proposed stack maps cleanly onto Gat's three layers plus the always-on safety core that every successful flight system kept:
  - Mission = deliberator (prediction, search, slow).
  - Skills = sequencer (memory of the past, conditional, no search).
  - Moves = controller primitives (ArduPilot modes: bounded-time, stateless or ephemeral, continuous).
  - Reflexes = Alfred's always-running safety routine, ASE's "independent layer of fault protection", and RAX's conventional flight software beneath the AI.
- Where failures concentrate (Remote Agent): the multi-threaded executive. My opinion: Skills should be single-threaded or event-loop based per vehicle, with explicit state machines or behavior trees that can be model-checked or exhaustively tested. Concurrency between Skills should be avoided rather than managed.
- RAX's second anomaly (RA's belief inconsistent with spacecraft state) is Gat's "state loses sync with reality" in practice. Skills should re-read autopilot mode and state on every tick and treat any discrepancy as a cognizant failure, not assume their commanded mode took effect.
- CLARAty's merge of planner and executive is a warning not to over-separate Mission from Skills for a small team. A thin Mission layer that is mostly configuration plus a few query-style planners (ATLANTIS style: the sequencer asks the deliberator) is probably enough. That is opinion.
- Gat's Alfred, 3T at JSC and ASE all show small teams and small compute succeeding when the middle layer is the main investment and the deliberator is simple.

### Gaps
- I did not read the full text of "Three years in the trenches" (3T failure specifics) or the Havelund post-flight analysis. I recall that the post-flight analysis found the in-flight deadlock matched a pattern found pre-flight in another module, but I could not verify this, so it is not stated as fact.
- No independent evaluation of ASE's 100x claim was found; it is self-reported.
- I found no authoritative post-mortem or critique of 4D/RCS (e.g., why it was not widely adopted outside Army/NIST programmes).
- Not covered: LAAS/GenoM, CMU TCA/TDL, and Soar/ACT-R on robots (see Q5).

---

## Q5. Robotics: basal-ganglia-inspired action selection and cognitive architectures on robots

### Takeaway
BG-inspired selection has been shown on small lab robots (Khepera foraging, rat-like behaviours): selection works cleanly across sensory and motivational conditions, and the circuit centralises arbitration. These systems have not been deployed in field robots. Cognitive architectures are many (84 surveyed) and used in 900+ projects, but the survey reports a gap with mainstream robotics and lower range and efficiency for biologically inspired models.

### Cited Findings
- Prescott, Montes González, Gurney et al. 2006, "A robot model of the basal ganglia: Behavior and intrinsic processing", *Neural Networks* 19(1):31-61. A computational BG model was embedded in a robot doing an animal-inspired selection task, and the authors report effective selection across a wide range of sensory and motivational conditions. — [White Rose eprint](https://eprints.whiterose.ac.uk/107037)
- The robot behaviours were modelled on a lab rat's home-cage activities on a Khepera robot (Khepera vs. Khepera II differs by source). A 2002 *Basal Ganglia VII* chapter used a mock foraging task under changing sensory and motivational conditions. — [White Rose 155190](https://eprints.whiterose.ac.uk/155190); [Scitepress 2012 citing paper](https://scitepress.org/Papers/2012/37336/pdf/index.html)
- A 2024 *Biomimetics* follow-up (Prescott, Montes González, Gurney, Humphries, Redgrave) uses the same robot model to vary tonic simulated dopamine in a foraging task. Full text was blocked by a CAPTCHA, so results are not verified. — [PMC10967936](https://pmc.ncbi.nlm.nih.gov/articles/PMC10967936)
- Kotseruba & Tsotsos (*Artificial Intelligence Review* 2018) surveyed 84 cognitive architectures (49 active) and 900+ application projects. The arXiv version notes an "apparent gap" between cognitive-architecture research and general robotics/vision research. It adds that biologically inspired models "do not have the same range and efficiency" as engineering-based systems, and that only about one-third of the architectures are open source. — [arXiv 1610.08602](https://arxiv.org/abs/1610.08602v2)

### Inferences
- The transferable idea from BG models is architectural, not neural: one central selector owns access to a contested resource (here, the autopilot's command channel and the gimbal), and candidates bid with salience. This contrasts with subsumption's distributed wiring. Redgrave et al.'s argument for a central switch is about efficiency, clean switching and avoiding conflict over shared resources.
- For a small team, a BG-style selector can be as simple as "the Skill executive owns MAVLink commanding; Skills request, the executive grants one at a time with explicit priority and hysteresis". That is opinion. Gat's Alfred shows the hysteresis idea: react fast to danger, resume slowly.
- Cognitive architectures (Soar, ACT-R, etc.) have no documented path to small fixed-wing UAV autonomy in the sources found. My opinion: they are out of scope for a small team.

### Gaps
- No quantitative comparison was found of BG-model selection vs. simple winner-take-all or priority arbitration on robots (dithering, switching latency).
- No sources were retrieved on Soar/ACT-R deployments on UAVs (e.g., TacAir-Soar in simulation) or on Psikharpax/Girard et al. BG work.

---

## Q6. Safety overrides: how do deployed systems keep the "Reflexes" layer simple and verifiable? (Simplex / run-time assurance, Auto-GCAS, ASTM F3269, ICAROUS, ArduPilot Advanced Failsafe)

### Takeaway
The proven pattern is Simplex / run-time assurance. A complex, unverified primary controller runs freely, while a small verified monitor plus a simple recovery controller takes over when the state nears a safety boundary, then hands back. Auto-GCAS on F-16s is the flagship deployment: it was fielded around 2014 on 600+ aircraft and credited with 8-10+ saved pilots. ArduPilot already has a hardware-backed version of the bottom of this stack (Advanced Failsafe with IOMCU termination).

### Cited Findings
- RTA "filters" an unverified primary controller (human, advanced or autonomous) through a monitor plus backup controller that replaces or modifies inputs when needed. It is agnostic to the primary controller's internals, which "decouples safety enforcement from performance objectives". — [Hobbs et al. 2021/22, arXiv 2110.03506](https://arxiv.org/abs/2110.03506)
- Sha's Simplex ("Using simplicity to control complexity", IEEE Software 2001) switches between a high-assurance and a high-performance control subsystem. The core argument is that reliability comes from a simple, dependable core that guarantees critical functions, rather than from diversity, because complexity breeds bugs, bugs are not equally critical, and budgets are finite. — [Purdue reading-group slides](https://engineering.purdue.edu/dcsl/reading/2007/foob-using_simplicity_to_control_complexity.pdf); [Crenshaw 2008 dissertation](https://www.ideals.illinois.edu/items/11503)
- In Simplex variants, a verified decision module switches discretely to a backup controller whose safety is proven (e.g., by barrier certificates, from which cheap switching conditions can be derived). Some work replaces static verification of the baseline with runtime checks (attributions within the result set are uncertain). — [Stony Brook RV 2022 paper](https://www3.cs.stonybrook.edu/%7Estoller/papers/rv2022.pdf); [NSF PAR](https://par.nsf.gov/biblio/10327798)
- ASTM F3269 (2017; revised 2021) is a standard practice for showing a civil aviation authority that a UAS's complex functions are bounded by an RTA architecture. A safety monitor checks behaviour against predefined limits and engages a recovery function, as an alternative to design-time assurance (DO-178C etc.) for complex functions. — [ASTM F3269-17](https://store.astm.org/f3269-17.html)
- A NASA/Collins demonstration built an F3269-17-based architecture with diverse runtime monitors and formally synthesised high-assurance components. It stayed safe despite defects in the learning-enabled component. — [NTRS 20200001756](https://ntrs.nasa.gov/citations/20200001756)
- Auto-GCAS (F-16): the system warns the pilot. If there is no response, it locks out pilot inputs, performs "an abrupt roll-to-upright and a nominal 5-G pull until terrain clearance is assured", and returns control. Controlled flight into terrain caused 75% of F-16 crashes. — [The Aviationist 2015](https://theaviationist.com/2015/02/02/f-16-gcat-explained/); [AOPA 2016](https://www.aopa.org/news-and-media/all-news/2016/september/21/automation-saves-another-pilot)
- Auto-GCAS was fielded on F-16 Block 40/50 starting in 2014 (installs reportedly began in 2013) and credited with eight pilots saved in about 4.5 years per NASA (2019), with later sources citing 10. It runs on 600+ USAF F-16s and the team won the 2018 Collier Trophy. — [NASA Armstrong 2019](https://www.nasa.gov/centers/armstrong/news/newsreleases/2019/19-02.html); [Lockheed Martin](https://lockheedmartin.com/en-us/products/autogcas.html); [Edwards AFB](https://www.edwards.af.mil/News/Article/932081/416th-flts-testers-meet-with-auto-gcas-survivor/)
- NASA ICAROUS: its core autonomy functions are constraint-conformance monitoring and contingency control. It computes resolution and recovery manoeuvres "autonomously executed by the autopilot" when safety criteria are, or are about to be, violated. It is built as cFS apps on a publish/subscribe bus. Runtime monitors generated from FRET requirements via Copilot run inside ICAROUS in flight. — [NASA LaRC ICAROUS page](https://shemesh.larc.nasa.gov/fm/ICAROUS); [NTRS 20205010646](https://ntrs.nasa.gov/citations/20205010646); [NTRS 20200000072](https://ntrs.nasa.gov/api/citations/20200000072/downloads/20200000072.pdf)
- ArduPilot Plane Advanced Failsafe (AFS) was built for the Outback Challenge, to keep the aircraft inside a defined airspace even by deliberate termination. Triggers:
  - geofence breach and AFS_MAX_RANGE: immediate termination;
  - pressure altitude above AFS_AMSL_LIMIT, or barometer unhealthy for 5 s without GPS margin: termination;
  - GPS lost for 3 s: jump to a configured mission waypoint;
  - GCS heartbeat lost (default 10 s): jump to a configured waypoint;
  - dual GPS+GCS loss (optional) or RC loss beyond a timeout: termination;
  - manual AFS_TERMINATE.

  Termination is "not recoverable". When enabled, AFS configures the secondary IO microcontroller to terminate if communication with the main FMU stops (e.g., firmware crash). The docs say nothing about companion computers. — [ArduPilot AFS docs](https://ardupilot.org/plane/docs/advanced-failsafe-configuration.html)
- ArduPilot supports companion computers for navigation needing more compute. ICAROUS documentation recommends MAVProxy with custom modules for geofence upload. Whether ICAROUS was flown on ArduPilot is not confirmed by an official source (only a forum remark). — [ICAROUS GitHub fork](https://github.com/5nefarious/icarous); [ArduPilot (Wikipedia)](https://en.wikipedia.org/wiki/ArduPilot)

### Inferences
- The proposed "Reflexes always override" layer is a Simplex/RTA safety controller. The literature's design rules (my synthesis):
  1. The monitor and recovery behaviours must be much simpler than what they guard, and verifiable on their own.
  2. Switching is discrete and explicit, not blended.
  3. Recovery is a small set of pre-validated manoeuvres (Auto-GCAS: roll upright plus pull; ArduPilot: RTL, LOITER, land, terminate).
  4. Hand-back is explicit and conditional: control is returned only once the state is clear, with hysteresis.
  5. The safety core must not depend on the complex component's health (AFS's IOMCU watchdog; ASE's independent fault protection).
- For ArduPilot plus a companion computer, this implies the Reflexes should live on the flight controller (failsafes, geofence, AFS, possibly Lua-scripted monitors) and not on the companion. The companion's Skills are the "unverified primary controller". A "supervisor hold" on the companion is a convenience reflex, not a safety reflex, because the companion can crash. That is opinion consistent with Simplex principles.
- ICAROUS is the closest published analogue to "Skills on a companion computer bounded by monitors". Its use of formally generated runtime monitors (FRET/Copilot) is a model for making the supervisor-hold logic specifiable and testable.

### Gaps
- The full text of Hobbs et al. (RTA tutorial) could not be fetched (PDF too large), so detailed RTA design guidance (recovery regions, latching, chattering avoidance) is from general knowledge and abstracts only.
- The ASTM F3269 text itself was not accessible.
- No official confirmation was found that ICAROUS has flown on ArduPilot, or of the current Auto-GCAS lives-saved count (figures date from 2016-2020).

---

## Q7. Lessons for a small team building fixed-wing UAV autonomy on ArduPilot with a companion computer: what to keep layered, where to put arbitration, how to keep safety overrides simple and verifiable

### Takeaway
The proposed Mission -> Skills -> Moves -> Reflexes stack matches both the biological pattern (select and parameterise self-stabilising lower loops; protective reflexes that are always on and context-gated) and the most successful robot pattern (Gat's three layers plus an independent, simple safety core, as in ASE, RAX and Simplex/Auto-GCAS). The main risks the literature points to:
- arbitration spread across layers (the subsumption failure);
- a concurrent, stateful middle layer drifting out of sync with the vehicle (the Remote Agent failure);
- safety logic hosted on the complex component.

### Cited Findings
- Three-layer guidance:
  - The controller fails "cognizantly" and keeps only ephemeral state.
  - The sequencer owns transitions between continuous regimes and does no search.
  - The deliberator may be trivial.
  - Small teams can field it quickly: Alfred's contest code took three days for one person. — [Gat 1998](https://flownet.com/gat/papers/tla.pdf)
- Subsumption's lack of modularity came from upper layers interfering with lower layers' internals. Tooth and Rocky III, where upper layers gave "input or advice" (parameters) instead of overriding outputs, were highly reliable. — [Gat 1998, quoting Hartley & Pipitone 1991](https://flownet.com/gat/papers/tla.pdf)
- A centralised selector for contested resources has efficiency advantages over distributed selection. — [Redgrave, Prescott & Gurney 1999](https://org.osu.edu/cognitive-science-club/files/2020/01/RedgravePrescottGurney99.pdf)
- The DS1 executive deadlock (missing critical section between two threads) was not seen in thousands of ground runs. Model checking found five concurrency errors pre-flight that testing would likely have missed. The flight project refused an untested patch. — [Nayak et al. 1999](https://ai.jpl.nasa.gov/public/documents/papers/rax-results-isairas99.pdf); [Havelund et al.](https://ntrs.nasa.gov/api/citations/20000055731/downloads/20000055731.pdf)
- The successful long-running onboard autonomy (ASE, 12+ years) kept the flight software with "an independent layer of fault protection" beneath planner and executive. — [Sherwood et al. 2006](https://ai.jpl.nasa.gov/public/papers/sherwood-spaceops06-autonomy.pdf)
- ArduPilot AFS provides geofence and altitude termination with an IOMCU watchdog independent of the main FMU firmware. — [ArduPilot AFS docs](https://ardupilot.org/plane/docs/advanced-failsafe-configuration.html)
- In biology, higher levels set feedback gains and goals per task (OFC) and gate reflex gain by phase and task (presynaptic inhibition, reflex reversal) rather than disabling reflexes. — [Scott 2002](https://homes.cs.washington.edu/~todorov/papers/ScottNatNeurosci02_news.pdf); [Seki et al. 2003](https://www.nature.com/articles/nn1154); [Lamont & Zehr 2006](https://link.springer.com/doi/10.1007/s00221-006-0586-4)

### Inferences (all are opinion/synthesis, offered for the report writer to weigh)
1. **Keep four layers, and give each a different state discipline (Gat).**
   - Reflexes: stateless or ephemeral, always running.
   - Moves: ArduPilot modes, bounded-time, continuous.
   - Skills: memory of the past, conditional sequencing, no search.
   - Mission: prediction and search, asynchronous, may be slow.

   This gives a testable rule for where new code goes.
2. **Moves are primitives, so parameterise them, don't micromanage.** As with brainstem drive to CPGs and cortical gain-setting, Skills should mostly select a Move and set its parameters (target, loiter radius, altitude band, airspeed) and let ArduPilot close the loop. A Skill that must stream commands itself (e.g., vision final approach via GUIDED/attitude targets) is the exception. It should be bounded by the Move's envelope and the Reflexes, like fine cortical control riding on top of spinal reflexes.
3. **Put arbitration in exactly one place per resource (BG lesson vs. subsumption failure).** A single Skill executive per vehicle owns the MAVLink command channel (and the gimbal). Skills request and the executive grants one at a time by explicit priority, with hysteresis: fast to yield to danger, slow to resume (Alfred). Avoid suppression wires or vector-blending across Skills.
4. **Reflexes sit outside arbitration and outside the companion (Simplex).**
   - Hard reflexes (geofence, RC/GCS/battery failsafes, AFS termination, altitude limits) live on the flight controller, are never suppressible from above, and switch discretely to pre-validated recovery Moves (RTL, LOITER, land, terminate).
   - A companion-side "supervisor hold" is a soft reflex, useful but not safety-critical, because the companion is the unverified component.
   - Hand-back after a reflex should require an explicit, logged re-grant by the Skills layer. Don't resume the previous Skill automatically.
5. **Allow context-gated soft reflexes only in declared, bounded, logged ways (biology's gain modulation).** If a Skill needs a reflex tuned (e.g., a final-approach Skill legitimately operating near a dock), the Skill should declare it up front with bounds. The Reflex layer enforces the bounds, and the change expires when the Skill ends (Gat's ephemeral state).
6. **Fail cognizantly and re-sync every tick (RAX lesson).** Each Move and Skill must detect and report its own failure (mode not accepted, target unreachable, POI lost). Skills must re-read actual autopilot mode and state every cycle rather than trust their own record of what they commanded.
7. **Keep the Skills layer single-threaded per vehicle and statically analysable.** Use explicit state machines or behavior trees, which survey evidence prefers over ad-hoc FSMs as complexity grows. Concurrency in the executive caused the best-known autonomy flight failure.
8. **Don't over-build Mission.** CLARAty merged planner and executive, Gat says the deliberator can be "trivial", and Alfred's planner was exhaustive search over a small space. For a small team, Mission can be configuration plus a few query-style planners that Skills call (ATLANTIS style).
9. **Make the safety argument checkable.** Express supervisor and reflex rules as formal or declarative requirements with generated monitors (the ICAROUS FRET/Copilot approach) or at least table-driven rules with exhaustive tests. Test in a pyramid from fast SITL to hardware (RAX).
10. **NavPy-specific note.** Under the project's Pure Vision Approach constraint, the final-approach Skill's command path may not consume compass yaw, ground speed or altitude-derived vertical state. The layering above is compatible with that: those quantities can still feed Reflexes and Mission (allowed outside the approach command path). Keeping the Reflexes on the flight controller cleanly separates "safety may use GPS and altitude" from "approach command must not".

### Gaps
- No source specifically evaluates layered autonomy architectures on small fixed-wing UAVs running ArduPilot plus a companion computer. ICAROUS and WITAS/HDRC3 are the closest, and neither published a fixed-wing ArduPilot field record in the sources found.
- No quantitative data was found comparing arbitration schemes (central selector vs. priority suppression vs. blending) on flying vehicles.
- Fixed-wing-specific constraints (minimum airspeed, turn radius, no hover) for reflex recovery design were not covered by any retrieved architecture source. This needs aerodynamics and ArduPilot-specific input.
