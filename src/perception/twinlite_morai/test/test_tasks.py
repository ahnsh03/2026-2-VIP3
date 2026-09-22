import unittest

import torch

from twinlite_morai.tasks import (
    BINARY_LANE_TASK,
    ROAD_MARKING_TASK,
    configure_twinlite_task,
    load_task_compatible_state_dict,
    task_from_checkpoint,
    task_from_training_heads,
)


class FakeUpConvBlock(torch.nn.Module):
    def __init__(self, in_channels, out_channels, last=False):
        super().__init__()
        self.up_conv = torch.nn.Module()
        self.up_conv.deconv = torch.nn.ConvTranspose2d(
            in_channels, out_channels, kernel_size=1
        )
        self.last = bool(last)


class FakeTwinLite(torch.nn.Module):
    def __init__(self):
        super().__init__()
        self.trunk = torch.nn.Conv2d(3, 8, kernel_size=1)
        self.out_ll = FakeUpConvBlock(8, 2, last=True)


class TaskContractTest(unittest.TestCase):
    def test_training_head_order_is_canonicalized(self):
        self.assertEqual(
            task_from_training_heads(("road_marking", "drivable")),
            ROAD_MARKING_TASK,
        )

    def test_legacy_checkpoint_defaults_to_binary_lane(self):
        self.assertEqual(task_from_checkpoint({"args": {}}), BINARY_LANE_TASK)

    def test_configure_replaces_only_secondary_output_block(self):
        model = FakeTwinLite()
        original = model.out_ll
        reset = configure_twinlite_task(model, ROAD_MARKING_TASK)
        self.assertEqual(reset, ("out_ll",))
        self.assertIsNot(model.out_ll, original)
        self.assertEqual(model.out_ll.up_conv.deconv.in_channels, 8)
        self.assertEqual(model.out_ll.up_conv.deconv.out_channels, 4)
        self.assertTrue(model.out_ll.last)
        self.assertEqual(configure_twinlite_task(model, ROAD_MARKING_TASK), ())

    def test_binary_checkpoint_transfer_preserves_trunk_and_resets_output(self):
        source = FakeTwinLite()
        with torch.no_grad():
            source.trunk.weight.fill_(0.25)
            source.out_ll.up_conv.deconv.weight.fill_(0.75)
        source_state = source.state_dict()

        target = FakeTwinLite()
        reset = configure_twinlite_task(target, ROAD_MARKING_TASK)
        output_before = target.out_ll.up_conv.deconv.weight.detach().clone()
        report = load_task_compatible_state_dict(
            target,
            source_state,
            BINARY_LANE_TASK,
            ROAD_MARKING_TASK,
            reset,
        )

        self.assertEqual(report["transfer"], "binary_lane_to_road_marking")
        self.assertTrue(torch.all(target.trunk.weight == 0.25))
        self.assertTrue(
            torch.equal(target.out_ll.up_conv.deconv.weight, output_before)
        )


if __name__ == "__main__":
    unittest.main()
