\subsection{Path Searching}

Some words might be appropriate describing equation~(\ref{eq:sample}), if 
we had but time and space enough. 

\begin{equation} \label{eq:sample}
{{\partial F}\over {\partial t}} = D{{\partial^2 F}\over {\partial x^2}}.
\end{equation}

See \cite{Abl:56}, \cite{AbTaRu:54}, \cite{Keo:58} and \cite{Pow:85}.

\subsubsection{Example.} This equation goes far beyond the
celebrated theorem ascribed to the great Pythagoras by his followers.

\begin{thm}   % use the thm environment for theorems
The square of the length of the hypotenuse of a right triangle equals
the sum of the squares of the lengths of the other two sides.
\end{thm}

\begin{pf}    % and the pf environment for proofs
The square of the length of the hypotenuse of a right triangle equals the sum of the squares 
of the lengths of the other two sides.
\end{pf}

%% There are a number of predefined theorem-like environments in
%% ifacconf.cls:
%%
%% \begin{thm} ... \end{thm}            % Theorem
%% \begin{lem} ... \end{lem}            % Lemma
%% \begin{claim} ... \end{claim}        % Claim
%% \begin{conj} ... \end{conj}          % Conjecture
%% \begin{cor} ... \end{cor}            % Corollary
%% \begin{fact} ... \end{fact}          % Fact
%% \begin{hypo} ... \end{hypo}          % Hypothesis
%% \begin{prop} ... \end{prop}          % Proposition
%% \begin{crit} ... \end{crit}          % Criterion

Of course LaTeX manages equations through built-in macros. You may
wish to use the \texttt{amstex} package for enhanced math
capabilities.

\subsection{Trajectory Following}

To insert figures, use the \texttt{graphicx} package. Although other
graphics packages can also be used, \texttt{graphicx} is simpler to
use. See  Fig.~\ref{fig:bifurcation} for an example.

\begin{figure}
\begin{center}
\includegraphics[width=8.4cm]{bifurcation}    % The printed column width is 8.4 cm.
\caption{Bifurcation: Plot of local maxima of $x$ with damping $a$ decreasing} 
\label{fig:bifurcation}
\end{center}
\end{figure}

Figures must be centered, and have a caption at the bottom. 

\subsection{Tables}
Tables must be centered and have a caption above them, numbered with
Arabic numerals. See table~\ref{tb:margins} for an example.

\begin{table}[hb]
\begin{center}
\caption{Margin settings}\label{tb:margins}
\begin{tabular}{cccc}
Page & Top & Bottom & Left/Right \\\hline
First & 3.5 & 2.5 & 1.5 \\
Rest & 2.5 & 2.5 & 1.5 \\ \hline
\end{tabular}
\end{center}
\end{table}

\subsection{Final Stage}

Authors are expected to mind the margins diligently.  Papers need to
be stamped with event data and paginated for inclusion in the
proceedings. If your manuscript bleeds into margins, you will be
required to resubmit and delay the proceedings preparation in the
process.

\subsubsection{Page margins.} See table~\ref{tb:margins} for the
page margins specification. All dimensions are in \emph{centimeters}.


\subsection{PDF Creation}

All fonts must be embedded/subsetted in the PDF file. Use one of the
following tools to produce a good quality PDF file:

\subsubsection{PDFLaTeX} is a special version of LaTeX by Han The
Thanh which produces PDF output directly using Type-1 fonts instead of
the standard \texttt{dvi} file. It accepts figures in JPEG, PNG, and PDF
formats, but not PostScript. Encapsulated PostScript figures can be
converted to PDF with the \texttt{epstopdf} tool or with Adobe Acrobat
Distiller.

\subsubsection{Generating PDF from PostScript} is the classical way of
producing PDF files from LaTeX. The steps are:

\begin{enumerate}
  \item Produce a \texttt{dvi} file by running \texttt{latex} twice.
  \item Produce a PostScript (\texttt{ps}) file with \texttt{dvips}.
  \item Produce a PDF file with \texttt{ps2pdf} or Adobe Acrobat
  Distiller.
\end{enumerate}

\subsection{Copyright Form}

IFAC will put in place an electronic copyright transfer system in due
course. Please \emph{do not} send copyright forms by mail or fax. More
information on this will be made available on IFAC website.


\section{NAVIGATION FRAMEWORK}



\section{Navigation System}
\subsection{Terrain Traversability Assessment Algorithm}
This section details the terrain traversability assessment algorithm, which generates traversability maps and identifies low-traversability corridors (LTCs) for robotic navigation.

\subsubsection{Overview}
The algorithm integrates spatial partitioning, clustering, and cost-based classification to process LiDAR data. The pipeline outputs traversability maps at 10 cm resolution and LTC contours for safe path planning.

\subsubsection{Point Cloud Preprocessing and Spatial Partitioning}
For each LiDAR frame, a polar grid is constructed with dynamic radii $r \in [0.5\,\mathrm{m}, 30\,\mathrm{m}]$ (exponentially spaced) and $N=36$ angular sectors (10° each). Each sector $C_i$ is defined as:
\begin{equation}
\theta_i = \left[ \frac{2\pi(i-1)}{N}, \frac{2\pi i}{N} \right], \quad r_i = [r_{k-1}, r_k]
\label{eq:polar_grid}
\end{equation}
where $k$ denotes the radial layer (see Fig.~\ref{fig:grid}).

\subsubsection{Clustering and Terrain Classification}
\begin{enumerate}
    \item \textbf{Coordinate System Alignment}: Point cloud data is transformed into the sensor's local coordinate frame.
    \item \textbf{Point Cloud Clustering}: DBSCAN ($\epsilon=0.5\,\mathrm{m}$, $\mathrm{minPts}=5$) segments obstacles from terrain.
    \item \textbf{Plane Fitting}: RANSAC estimates plane equations with $\epsilon_{\text{plane}}=0.1\,\mathrm{m}$ and $\tau_{\text{num}}=15$ points.
\end{enumerate}

\subsubsection{Traversability Assessment and Cost Function Design}
For each plane $P$:
\begin{enumerate}
    \item \textbf{Normal Similarity}: 
    \begin{equation}
    S_{\text{normal}} = \frac{\mathbf{n}_P \cdot \mathbf{n}_{\text{horizontal}}}{||\mathbf{n}_P||}
    \label{eq:normal_similarity}
    \end{equation}
    where $\mathbf{n}_{\text{horizontal}} = [0,0,1]^T$.
    
    \item \textbf{Height Variation}: 
    \begin{equation}
    \sigma_{h} = \sqrt{\frac{1}{N_{\text{points}}} \sum_{p \in P} (h_p - \bar{h})^2}, \quad \bar{h} = \frac{1}{N_{\text{points}}} \sum h_p
    \label{eq:height_variation}
    \end{equation}
    
    \item \textbf{Cost Function}: 
    \begin{equation}
    C_{\text{cost}} = 0.6(1 - S_{\text{normal}}) + 0.4 \cdot \frac{\sigma_h}{0.3\,\mathrm{m}}
    \label{eq:cost_function}
    \end{equation}
    
    \item \textbf{Level Mapping}: 
    \begin{equation}
    L = \min\left(10, \left\lfloor 10 \cdot \frac{C_{\text{cost}}}{1.0} \right\rfloor + 1 \right)
    \label{eq:level_mapping}
    \end{equation}
\end{enumerate}

\subsubsection{Low-Traversability Region Clustering and Merging}
\begin{enumerate}
    \item \textbf{Thresholding}: Regions with $L \geq 7$ are marked as LTC candidates.
    \item \textbf{Spatial Clustering}: Region-growing merges adjacent regions with $\Delta C_{\text{thres}}=0.2$.
    \item \textbf{Merging}: Proximity-based merging ($d_{\text{merge}}=0.5\,\mathrm{m}$) forms LTC polygons.
\end{enumerate}

\subsubsection{Algorithm Pipeline}
The complete workflow is summarized as:
\begin{enumerate}
    \item Input LiDAR data and align coordinates.
    \item Partition into polar sectors.
    \item Cluster points and classify terrain/obstacles.
    \item Compute $S_{\text{normal}}$, $\sigma_h$, and $C_{\text{cost}}$.
    \item Generate LTCs via clustering and merging.
    \item Output traversability maps and LTC contours.
\end{enumerate}


\subsection{Bidirectional Reachable-Set Search Algorithm for Rough Terrain Planning}
This section presents a bidirectional reachable-set search algorithm adapted for autonomous navigation in rough terrain environments, integrating terrain traversability assessment with dynamic path planning. The algorithm extends the principles from Section 2.1 by incorporating terrain-aware constraints and optimizing for safety in low-traversability regions.

\subsubsection{Algorithm Overview}
The algorithm employs forward-backward reachability analysis to generate dynamically feasible paths while respecting terrain constraints. Key adaptations include:
\begin{itemize}
    \item Terrain-derived constraints: Low-Traversability Corridors (LTCs) replace static obstacles, with time-varying spatial boundaries.
    \item Kinematic adaptation: Adjustments for planetary rover dynamics (e.g., reduced lateral velocity $v_{d_{\text{max}}}$ due to rough terrain).
    \item Cost function extension: Integration of terrain traversability costs with original smoothness and terminal constraints.
\end{itemize}

\subsubsection{Mathematical Formulation}
\begin{itemize}
    \item \textbf{Terrain Constraints}: For time $t$, define terrain boundaries using LTC polygons:
    \begin{equation}
    d_{\text{min}}(t) = \max\left(d_{\text{static}}, d_{\text{LTC}}(t)\right)
    \end{equation}
    \begin{equation}
    d_{\text{max}}(t) = \min\left(d_{\text{static}}, d_{\text{LTC}}(t)\right)
    \end{equation}
    
    \item \textbf{Reachable Set Propagation}: Modified lateral displacement constraints incorporating terrain slope $\alpha$:
    \begin{equation}
    \delta d = v_{d_{\text{max}}} \cdot \delta t \cdot \cos\alpha_{\text{max}}
    \end{equation}
    
    \item \textbf{Cost Function}: Augmented with terrain traversability cost $C_{\text{terrain}}$:
    \begin{equation}
    \text{total cost} = w_1 C_{\text{curvature}} + w_2 C_{\text{terminal}} + w_3 C_{\text{terrain}}
    \end{equation}
    where $C_{\text{terrain}} = \sum_{t} L(t) \cdot \Delta t$ integrates traversability levels from Eq.~\ref{eq:level_mapping}.
\end{itemize}

\subsubsection{Algorithm Implementation}
\begin{enumerate}
    \item \textbf{Forward Reachability Analysis}:
    \begin{itemize}
        \item Initialize with rover's current state $(d_{\text{start}}, t_0)$
        \item Propagate reachable sets considering:
        \begin{itemize}
            \item Kinematic constraints adjusted for terrain slope
            \item LTC polygons as time-varying obstacles
            \item Safe corridors derived from traversability levels
        \end{itemize}
        \item Classify reachable intervals:
        \begin{itemize}
            \item Type I: Never intersects high-risk LTCs ($L \geq 7$)
            \item Type II: Intersects or subsequent to intersecting critical zones
        \end{itemize}
    \end{itemize}
    
    \item \textbf{Backward Reachability Analysis}:
    \begin{itemize}
        \item Initialize from goal state, considering terminal traversability constraints
        \item Compute backward reachable sets with terrain-aware velocity limits
        \item Merge with forward sets to create bidirectional reachable intervals
    \end{itemize}
    
    \item \textbf{Path Generation}:
    \begin{itemize}
        \item Prioritize Type I intervals to minimize LTC intrusion
        \item Hybrid sampling strategy:
        \begin{itemize}
            \item Dense sampling near LTC boundaries ($\Delta d = 0.1$ m)
            \item Sparse sampling in safe regions ($\Delta d = 0.3$ m)
        \end{itemize}
        \item Adaptive horizon planning with terrain visibility constraints
    \end{itemize}
\end{enumerate}

\subsubsection{Terrain-Specific Adaptations}
\begin{itemize}
    \item \textbf{Slope Compensation}: Adjust lateral velocity limits using real-time slope estimates:
    \begin{equation}
    v_{d_{\text{max}}}(t) = v_{\text{nominal}} \cdot \sqrt{1 - (\tan\alpha(t)/\mu_{\text{max}}})^2
    \end{equation}
    
    \item \textbf{Dynamic Corridors}: Time-varying safe corridors derived from LTC contours:
    \begin{equation}
    \text{Corridor}(t) = [d_{\text{LTC}_{\text{min}}}(t) + \delta_{\text{safe}}, d_{\text{LTC}_{\text{max}}}(t) - \delta_{\text{safe}}]
    \end{equation}
    where $\delta_{\text{safe}} = f(L(t))$ is safety margin dependent on traversability level.
    
\end{itemize}


\begin{figure}[htbp]
\centering
% \includegraphics[width=0.95\textwidth]{bidirectional_flowchart.pdf}
\caption{Algorithm workflow integrating terrain assessment with bidirectional search. Forward propagation (blue) considers terrain constraints, while backward propagation (red) ensures goal reachability. Merged bidirectional sets (purple) optimize path safety.}
\label{fig:algo_flow}
\end{figure}

This terrain-aware bidirectional search framework provides robust navigation capability for autonomous planetary exploration.


\subsection{Terrain-Aware Model Predictive Control for Trajectory Tracking}
This section presents a terrain-aware Model Predictive Control (MPC) framework that integrates traversability constraints and safety considerations derived from Section 3.1-3.2. The controller ensures precise trajectory tracking while dynamically adapting to terrain-induced kinematic constraints and avoiding low-traversability regions.

\subsubsection{MPC Formulation with Terrain Constraints}
The optimization problem at each time step 
k,k is formulated as:
\begin{aligned}
\min_{\mathbf{u}_{k:k+H-1}} \quad & \sum_{t=k}^{k+H-1} \left( \| \mathbf{x}_t - \mathbf{x}_t^{\text{ref}} \|_{\mathbf{Q}}^2 + \| \mathbf{u}_t \|_{\mathbf{R}}^2 + \alpha C_{\text{terrain}}(\mathbf{x}_t) \right) \\
& + \| \mathbf{x}_{k+H} - \mathbf{x}_{k+H}^{\text{ref}} \|_{\mathbf{P}}^2 \\
\text{s.t.} \quad & \mathbf{x}_{t+1} = f(\mathbf{x}_t, \mathbf{u}_t) + g(\alpha_{\text{terrain}}(\mathbf{x}_t)) \\
& \mathbf{x}_t \in \mathcal{X}_{\text{safe}}(t) \\
& \mathbf{u}_t \in \mathcal{U}_{\text{terrain}}(\mathbf{x}_t) \\
& \phi(\mathbf{x}_t) \cap \text{LTC}_i = \emptyset, \quad \forall i \in \mathcal{I}_{\text{LTC}}
\end{aligned}

\text{where:} \\
\begin{itemize}
\item $\mathbf{x}_t \in \mathbb{R}^n$ and $\mathbf{u}_t \in \mathbb{R}^m$ denote the state and control input at time $t$.
\item $H$ is the prediction horizon.
\item $f(\cdot)$ represents the nominal kinematic model modified by terrain slope effects $g(\cdot)$.
\item $\mathcal{X}_{\text{safe}}$ incorporates bidirectional reachable sets from Section 3.2.3.
\item $\mathcal{U}_{\text{terrain}}$ enforces slope-dependent velocity constraints (Eq.~\ref{eq:slope_compensation}).
\end{itemize}


\subsubsection{Terrain-Adaptive Vehicle Model}
The state propagation uses a modified unicycle model with terrain compensation:

\begin{equation}
\begin{bmatrix}
\dot{x} \\
\dot{y} \\
\dot{\theta}
\end{bmatrix}
=
\begin{bmatrix}
v \cos\theta \cdot \eta(\alpha) \\
v \sin\theta \cdot \eta(\alpha) \\
\omega
\end{bmatrix}
+
\begin{bmatrix}
w_x \\
w_y \\
w_\theta
\end{bmatrix},
\label{eq:terrain_model}
\end{equation}
where $\eta(\alpha)$ accounts for slope-induced velocity reduction, and $\mathbf{w} = [w_x, w_y, w_\theta]^T$ represents terrain roughness disturbances bounded by $\| \mathbf{w} \|_\infty \leq \beta L(\mathbf{x})$.
\begin{equation}
C_{\text{terrain}} = \underbrace{\sum_{i=1}^3 \lambda_i L(\mathbf{x})}_{\text{Traversability Level}} + \underbrace{\gamma |\nabla L(\mathbf{x})|}_{\text{Gradient Penalty}} + \underbrace{\kappa d_{\text{LTC}}^{-2}}_{\text{Safety Margin}},
\label{eq:terrain_cost}
\end{equation}
where:
\begin{itemize}
\item $L(\mathbf{x})$ is the traversability level from Eq.~\ref{eq:level_mapping}.
\item $\nabla L(\mathbf{x})$ penalizes abrupt terrain transitions.
\item $d_{\text{LTC}}$ denotes the distance to the nearest line terrain constraint (LTC) boundary.
\end{itemize}

\subsubsection{Constraint Handling Strategy}
\begin{enumerate}
\item \textbf{Safe Corridor Constraints}: Enforce path containment within bidirectional reachable sets:
\begin{equation}
\mathbf{x}_t \in \bigcup{j=1}^N [\underline{d}_j(t), \overline{d}_j(t)] \times [\underline{v}_j(t), \overline{v}_j(t)]
\end{equation}

\item \textbf{Dynamic Input Constraints}: Adapt control limits using real-time terrain data:
\begin{equation}
v_{\text{max}}(\mathbf{x}) = v_{\text{nominal}} \cdot \left(1 - \frac{L(\mathbf{x})}{10}\right)^{0.5}
\end{equation}

\item \textbf{Slope-Dependent Friction Cone Constraints}:
\begin{equation}
\begin{bmatrix}
a_x \ a_y
\end{bmatrix}
\leq \mu g \cos\alpha - g \sin\alpha
\end{equation}
\end{enumerate}

\subsubsection{Real-Time Implementation}
The nonlinear optimization problem is solved using a real-time iteration scheme:
\begin{enumerate}
\item \textbf{Linearization}: Jacobian matrices computed using terrain gradient information
\begin{equation}
\mathbf{A}t = \left.\frac{\partial f}{\partial \mathbf{x}}\right|{\mathbf{x}_t, \alpha_t}, \quad \mathbf{B}t = \left.\frac{\partial f}{\partial \mathbf{u}}\right|{\mathbf{x}_t, \alpha_t}
\end{equation}

\item \textbf{Convexification}: Convert LTC avoidance constraints to signed distance inequalities
\begin{equation}
\phi(\mathbf{x}_t) = \min_i \text{dist}(\mathbf{x}_t, \text{LTC}i) \geq d{\text{safe}}
\end{equation}

\item \textbf{Warm Start}: Initialize with bidirectional search results from Section 3.2.3
\end{enumerate}

\subsubsection{Stability Analysis}
The closed-loop system satisfies the stability condition:
\begin{equation}
V(\mathbf{x}_{k+1}) - V(\mathbf{x}_k) \leq -\epsilon |\mathbf{x}_k - \mathbf{x}k^{\text{ref}}|^2 + \mathcal{O}(L{\text{max}})
\end{equation}
where 
V(x)=x^{⊤}P_x is the terminal cost function, and 
L_{max}bounds the terrain disturbance effects.

\subsubsection{Algorithm Summary}
The terrain-aware MPC executes the following steps at each control interval:
\begin{enumerate}
\item Receive updated traversability map and LTC contours
\item Generate initial guess using bidirectional reachable sets
\item Linearize dynamics model with current terrain parameters
\item Formulate and solve constrained QP approximation
\item Apply first control input and update safety constraints
\item Propagate terrain disturbance bounds for next iteration
\end{enumerate}

This framework enables robust trajectory tracking in rough terrain while maintaining formal safety guarantees through tight integration with the preceding terrain analysis and path planning modules.


\subsection{Figures and Tables}

Figure axis labels are often a source of confusion. Use words rather
than symbols. As an example, write the quantity ``Magnetization'', or
``Magnetization M'', not just ``M''. Put units in parentheses. Do not
label axes only with units.  For example, write ``Magnetization
($\mathrm{A}/\mathrm{m}$)'' or ``Magnetization ($\mathrm{A} \mathrm{m}^{-1}$)'', not just
 ``$\mathrm{A}/\mathrm{m}$''. Do not
label axes with a ratio of quantities and units. For example, write
``Temperature ($\mathrm{K}$)'', not ``$\mbox{Temperature}/\mathrm{K}$''.

Multipliers can be especially confusing. Write ``Magnetization
($\mathrm{kA}/\mathrm{m}$)'' or ``Magnetization ($10^3 \mathrm{A}/\mathrm{m}$)''. Do not write
``Magnetization $(\mathrm{A}/\mathrm{m}) \times 1000$'' because the reader would not know
whether the axis label means $16000\,\mathrm{A}/\mathrm{m}$ or $0.016\,\mathrm{A}/\mathrm{m}$.

\subsection{References}

Use Harvard style references (see at the end of this document). With
\LaTeX, you can process an external bibliography database 
using \texttt{bibtex},\footnote{In this case you will also need the \texttt{ifacconf.bst}
file, which is part of the \texttt{ifaconf} package.}
or insert it directly into the reference section. Footnotes should be avoided as
far as possible.  Please note that the references at the end of this
document are in the preferred referencing style. Papers that have not
been published should be cited as ``unpublished''.  Capitalize only the
first word in a paper title, except for proper nouns and element
symbols.

\subsection{Abbreviations and Acronyms}

Define abbreviations and acronyms the first time they are used in the
text, even after they have already been defined in the
abstract. Abbreviations such as IFAC, SI, ac, and dc do not have to be
defined. Abbreviations that incorporate periods should not have
spaces: write ``C.N.R.S.'', not ``C. N. R. S.'' Do not use abbreviations
in the title unless they are unavoidable (for example, ``IFAC'' in the
title of this article).

\subsection{Equations}

Number equations consecutively with equation numbers in parentheses
flush with the right margin, as in (\ref{eq:sample}).  To make your equations more
compact, you may use the solidus ($/$), the $\exp$ function, or
appropriate exponents. Use parentheses to avoid ambiguities in
denominators. Punctuate equations when they are part of a sentence, as
in

\begin{equation} \label{eq:sample2}
\begin{array}{ll}
\int_0^{r_2} & F (r, \varphi ) dr d\varphi = [\sigma r_2 / (2 \mu_0 )] \\
& \cdot \int_0^{\inf} exp(-\lambda |z_j - z_i |) \lambda^{-1} J_1 (\lambda  r_2 ) J_0 (\lambda r_i ) d\lambda 
\end{array}
\end{equation}

Be sure that the symbols in your equation have been defined before the
equation appears or immediately following. Italicize symbols ($T$
might refer to temperature, but T is the unit tesla). Refer to
``(\ref{eq:sample})'', not ``Eq. (\ref{eq:sample})'' or ``equation
(\ref{eq:sample})'', except at the beginning of a sentence: ``Equation
(\ref{eq:sample}) is \ldots''.

\subsection{Other Recommendations}

Use one space after periods and colons. Hyphenate complex modifiers:
``zero-field-cooled magnetization''. Avoid dangling participles, such
as, ``Using (1), the potential was calculated'' (it is not clear who or
what used (1)). Write instead: ``The potential was calculated by using
(1)'', or ``Using (1), we calculated the potential''.

A parenthetical statement at the end of a sentence is punctuated
outside of the closing parenthesis (like this). (A parenthetical
sentence is punctuated within the parentheses.) Avoid contractions;
for example, write ``do not'' instead of ``don' t''. The serial comma
is preferred: ``A, B, and C'' instead of ``A, B and C''.