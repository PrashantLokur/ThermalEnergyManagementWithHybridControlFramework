function parameter = prepParaVec(X, Tamb, TV,reset)

persistent Fp lastN lastHash
if nargin < 4, reset = false; end
if reset
    Fp = []; lastN = []; lastHash = [];
    fprintf('>> prepParaVec: persistent cache cleared.\n');
    parameter = [];  
    return;           
end
%% ---- Input validation ----
assert(numel(X)                    == 9,  'X must have 9 elements');
assert(isscalar(Tamb),                    'Tamb must be scalar');
assert(size(TV.states,     2)      == 9,  'TV.states must be N×9');
assert(size(TV.parameters, 2)      == 11, 'TV.parameters must be N×11');
assert(size(TV.ambient,    1)      == size(TV.parameters, 1), ...
       'TV.ambient and TV.parameters row count mismatch');

%% ---- State index mapping (defined once) ----

idxState = [8,  9,  5,  2,  1,  3,  5,  9,  6,  7,  6];

%% ---- Cache interpolants (persistent) ----

N = size(TV.parameters, 1);

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

    if ~isfinite(p(i))
        warning('prepParaVec: p%d not finite at (Tamb=%.2f, sVal=%.4f). Using nearest.', ...
                 i, Tamb, sVal);
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