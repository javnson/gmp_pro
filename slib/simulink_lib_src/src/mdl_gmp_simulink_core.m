
% Test Command
% [NumberUnpackedPorts, UnpackedDataTypes, UnpackedDataSizes,...
% NumberPackedPorts, PackedDataTypes, PackedDataSizes, Alignment] = ...
% mdl_gmp_simulink_core({'double'}, {[1]}, {'double', 'double'}, {[1], [1]}, 1)

function ...
    [NumberUnpackedPorts, UnpackedDataTypes, UnpackedDataSizes, ...
     NumberPackedPorts, PackedDataTypes, PackedDataSizes, ...
     Alignment, PackedBytes, UnpackedBytes] = mdl_gmp_simulink_core( ...
     MaskUnpackedDataTypes, MaskUnpackedDataSizes, ... 
     MaskPackedDataTypes, MaskPackedDataSizes,...
     MaskAlignment)

% MDL invoke parameters
% NumberUnpackedPorts, UnpackedDataTypes, UnpackedDataSizes, 
% NumberPackedPorts, PackedDataTypes, PackedDataSizes, 
% Alignment

%% Output, Unpack Port
% target: NumberUnpackedPorts, UnpackedDataTypes, UnpackedDataSizes, 
% source: MaskUnpackedDataTypes, MaskUnpackedDataSizes
if ~isa(MaskUnpackedDataTypes, 'cell')
    error(message('slrealtime:BytePacking:UnpackedCell'));
end

UnpackedDataTypes = [];
for j = 1 : length(MaskUnpackedDataTypes)
    mask_unpacked_data_type = MaskUnpackedDataTypes{j};

    if ~isa(mask_unpacked_data_type, 'char')
        error(message('slrealtime:BytePacking:UnpackedChar'));
    end
    if strcmp(mask_unpacked_data_type, 'double')
        unpacked_data_type = 0;
    elseif strcmp(mask_unpacked_data_type, 'single')
        unpacked_data_type = 1;
    elseif strcmp(mask_unpacked_data_type, 'int8')
        unpacked_data_type = 2;
    elseif strcmp(mask_unpacked_data_type, 'uint8')
        unpacked_data_type = 3;
    elseif strcmp(mask_unpacked_data_type, 'int16')
        unpacked_data_type = 4;
    elseif strcmp(mask_unpacked_data_type, 'uint16')
        unpacked_data_type = 5;
    elseif strcmp(mask_unpacked_data_type, 'int32')
        unpacked_data_type = 6;
    elseif strcmp(mask_unpacked_data_type, 'uint32')
        unpacked_data_type = 7;
    elseif strcmp(mask_unpacked_data_type, 'boolean')
        unpacked_data_type = 8;
    elseif strcmp(mask_unpacked_data_type, 'int64')
        unpacked_data_type = 9;
    elseif strcmp(mask_unpacked_data_type, 'uint64')
        unpacked_data_type = 10;
    else
        error(message('slrealtime:BytePacking:UnpackedTypes'));
    end
    % Parameter, UnpackedDataTypes, ok
    UnpackedDataTypes = [UnpackedDataTypes, unpacked_data_type]; %#ok
end

% Parameter, NumberUnpackedPorts, ok
NumberUnpackedPorts = length(UnpackedDataTypes);

UnpackedDataSizes = [];

if ~isa(MaskUnpackedDataSizes, 'cell')
    error(message('slrealtime:BytePacking:UnpackedSizeCell'));
end

for j = 1 : length(MaskUnpackedDataSizes)
    mask_unpacked_data_size = MaskUnpackedDataSizes{j};

    if ~isa(mask_unpacked_data_size, 'double')
        error(message('slrealtime:BytePacking:UnpackedSizeDouble'));
    end

    if (size(mask_unpacked_data_size, 1) > 1) || (size(mask_unpacked_data_size, 2) > 2)
        error(message('slrealtime:BytePacking:UnpackedSizeRow'));
    end

    if length(mask_unpacked_data_size) == 1
        if mask_unpacked_data_size < 1
            error(message('slrealtime:BytePacking:UnpackedSizeDim'));
        end
        UnpackedDataSizes = [UnpackedDataSizes, 1, mask_unpacked_data_size]; %#ok
    else
        if (mask_unpacked_data_size(1) < 1) || (mask_unpacked_data_size(2) < 1)
            error(message('slrealtime:BytePacking:UnpackedSizeTwoDim'));
        end
        UnpackedDataSizes = [UnpackedDataSizes, mask_unpacked_data_size]; %#ok
    end
end
if (NumberUnpackedPorts * 2) ~= length(UnpackedDataSizes)
    error(message('slrealtime:BytePacking:UnpackedSizeLength'));
end


%% Input, Pack Port
% target: NumberPackedPorts, PackedDataTypes, PackedDataSizes,
% source: MaskPackedDataTypes, MaskUnpackedDataSizes
if ~isa(MaskPackedDataTypes, 'cell')
    error(message('slrealtime:BytePacking:UnpackedCell'));
end

PackedDataTypes = [];
for j = 1 : length(MaskPackedDataTypes)
    mask_packed_data_type = MaskPackedDataTypes{j};

    if ~isa(mask_packed_data_type, 'char')
        error(message('slrealtime:BytePacking:UnpackedChar'));
    end
    if strcmp(mask_packed_data_type, 'double')
        packed_data_type = 0;
    elseif strcmp(mask_packed_data_type, 'single')
        packed_data_type = 1;
    elseif strcmp(mask_packed_data_type, 'int8')
        packed_data_type = 2;
    elseif strcmp(mask_packed_data_type, 'uint8')
        packed_data_type = 3;
    elseif strcmp(mask_packed_data_type, 'int16')
        packed_data_type = 4;
    elseif strcmp(mask_packed_data_type, 'uint16')
        packed_data_type = 5;
    elseif strcmp(mask_packed_data_type, 'int32')
        packed_data_type = 6;
    elseif strcmp(mask_packed_data_type, 'uint32')
        packed_data_type = 7;
    elseif strcmp(mask_packed_data_type, 'boolean')
        packed_data_type = 8;
    elseif strcmp(mask_packed_data_type, 'int64')
        packed_data_type = 9;
    elseif strcmp(mask_packed_data_type, 'uint64')
        packed_data_type = 10;
    else
        error(message('slrealtime:BytePacking:UnpackedTypes'));
    end
    % Parameter, PackedDataTypes, ok
    PackedDataTypes = [PackedDataTypes, packed_data_type]; %#ok
end

% Parameter, NumberPackedPorts, ok
NumberPackedPorts = length(PackedDataTypes);

PackedDataSizes = [];

if ~isa(MaskPackedDataSizes, 'cell')
    error(message('slrealtime:BytePacking:UnpackedSizeCell'));
end

for j = 1 : length(MaskPackedDataSizes)
    mask_packed_data_size = MaskPackedDataSizes{j};

    if ~isa(mask_packed_data_size, 'double')
        error(message('slrealtime:BytePacking:UnpackedSizeDouble'));
    end

    if (size(mask_packed_data_size, 1) > 1) || (size(mask_packed_data_size, 2) > 2)
        error(message('slrealtime:BytePacking:UnpackedSizeRow'));
    end

    if length(mask_packed_data_size) == 1
        if mask_packed_data_size < 1
            error(message('slrealtime:BytePacking:UnpackedSizeDim'));
        end
        PackedDataSizes = [PackedDataSizes, 1, mask_packed_data_size]; %#ok
    else
        if (mask_packed_data_size(1) < 1) || (mask_packed_data_size(2) < 1)
            error(message('slrealtime:BytePacking:UnpackedSizeTwoDim'));
        end
        PackedDataSizes = [PackedDataSizes, mask_unpacked_data_size]; %#ok
    end
end
if (NumberPackedPorts * 2) ~= length(PackedDataSizes)
    error(message('slrealtime:BytePacking:UnpackedSizeLength'));
end

%% Alignment
switch MaskAlignment
    case 1
        Alignment = 1;
    case 2
        Alignment = 2;
    case 3
        Alignment = 4;
    case 4
        Alignment = 8;
    otherwise
        error(message('slrealtime:BytePacking:Align'));
end

%% Packed ABI byte lengths used by the session handshake and network.json.
typeBytes = [8, 4, 1, 1, 2, 2, 4, 4, 1, 8, 8];
PackedBytes = 0;
for j = 1:NumberPackedPorts
    portBytes = prod(PackedDataSizes((j - 1) * 2 + (1:2))) * typeBytes(PackedDataTypes(j) + 1);
    PackedBytes = PackedBytes + portBytes + mod(-portBytes, Alignment);
end
UnpackedBytes = 0;
for j = 1:NumberUnpackedPorts
    portBytes = prod(UnpackedDataSizes((j - 1) * 2 + (1:2))) * typeBytes(UnpackedDataTypes(j) + 1);
    UnpackedBytes = UnpackedBytes + portBytes + mod(-portBytes, Alignment);
end

end
