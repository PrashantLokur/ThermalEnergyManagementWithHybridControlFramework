function parameter = prepParaVec(X, Tamb, TV,reset)
% Build 11×1 parameter vector using 2-D scattered interpolation
% over (ambient, chosen_state) -> param_i.
%
% Inputs:
%   X     [9×1 or 1×9]  current state vector
%   Tamb  [scalar]       ambient temperature in °C
%   TV    struct with fields:
%           .ambient     [N×1]   ambient temperature per sample
%           .states      [N×9]   state values per sample
%           .parameters  [N×11]  tuned parameter values per sample
%
% Mapping param_i → state index:
%   p1→x8, p2→x9, p3→x5, p4→x2, p5→x1,
%   p6→x3, p7→x5, p8→x9, p9→x6, p10→x7, p11→x6
persistent Fp lastN lastHash
if nargin < 4, reset = false; end
if reset
    Fp = []; lastN = []; lastHash = [];
    fprintf('>> prepParaVec: persistent cache cleared.\n');
    parameter = [];   % return empty, caller should check for this
    return;           % exit immediately, skip all other code
end
%% ---- Input validation ----
assert(numel(X)                    == 9,  'X must have 9 elements');
assert(isscalar(Tamb),                    'Tamb must be scalar');
assert(size(TV.states,     2)      == 9,  'TV.states must be N×9');
assert(size(TV.parameters, 2)      == 11, 'TV.parameters must be N×11');
assert(size(TV.ambient,    1)      == size(TV.parameters, 1), ...
       'TV.ambient and TV.parameters row count mismatch');

%% ---- State index mapping (defined once) ----
%        p1  p2  p3  p4  p5  p6  p7  p8  p9  p10 p11
idxState = [8,  9,  5,  2,  1,  3,  5,  9,  6,  7,  6];

%% ---- Cache interpolants (persistent) ----

N = size(TV.parameters, 1);

% Recompute if: first call, size changed, or TV content changed
% Use a lightweight hash: sum of first+last row of parameters
currentHash = sum(TV.parameters(1,:)) + sum(TV.parameters(end,:));

if isempty(Fp) || isempty(lastN) || lastN ~= N || ...
   isempty(lastHash) || lastHash ~= currentHash

    fprintf('>> Building interpolants (N=%d)...\n', N);

    amb = TV.ambient(:);    % [N×1]
    St  = TV.states;        % [N×9]
    Pm  = TV.parameters;    % [N×11]
    Fp  = cell(11, 1);

    for i = 1:11
        sj = idxState(i);
        xi = amb;           % ambient temperature
        yi = St(:, sj);     % driving state dimension
        zi = Pm(:, i);      % parameter values

        % Remove NaNs and duplicate points
        good = ~(isnan(xi) | isnan(yi) | isnan(zi));
        xi = xi(good);
        yi = yi(good);
        zi = zi(good);

        if length(xi) < 3
            error('prepParaVec: too few valid points for param p%d (got %d)', i, length(xi));
        end

        % 'linear' interpolation, 'nearest' extrapolation
        % (was 'nearest'/'nearest' before — 'linear' is more accurate inside hull)
        Fp{i} = scatteredInterpolant(xi, yi, zi, 'linear', 'nearest');
    end

    lastN    = N;
    lastHash = currentHash;
    fprintf('>> Interpolants ready.\n');
end

%% ---- Evaluate p1..p11 at (Tamb, X) ----
X = X(:);   % ensure column vector
p = zeros(11, 1);

for i = 1:11
    sVal = X(idxState(i));      % driving state value for param i
    p(i) = Fp{i}(Tamb, sVal);

    % Fallback: if still not finite, warn and use nearest neighbour explicitly
    if ~isfinite(p(i))
        warning('prepParaVec: p%d not finite at (Tamb=%.2f, sVal=%.4f). Using nearest.', ...
                 i, Tamb, sVal);
        % Force nearest by finding closest training point manually
        amb_col = TV.ambient(:);
        st_col  = TV.states(:, idxState(i));
        dist    = (amb_col - Tamb).^2 + (st_col - sVal).^2;
        [~, nn] = min(dist);
        p(i)    = TV.parameters(nn, i);
    end
end

%% ---- Pack output struct ----
pNames = {'p1','p2','p3','p4','p5','p6','p7','p8','p9','p10','p11'};
for i = 1:11
    parameter.(pNames{i}) = p(i);
end